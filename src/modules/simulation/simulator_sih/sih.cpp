/****************************************************************************
 *
 *   Copyright (c) 2019-2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file sih.cpp
 * Simulator in Hardware
 *
 * @author Romain Chiappinelli      <romain.chiap@gmail.com>
 *
 * Coriolis g Corporation - January 2019
 */

#include "aero.hpp"
#include "corridor_demo.hpp"
#include "sih.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_pwm_output.h>         // to get PWM flags
#include <lib/drivers/device/Device.hpp>

using namespace math;
using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor Sih::desc{task_spawn, custom_command, print_usage};

Sih::Sih() :
	ModuleParams(nullptr)
{
	srand(1234); // initialize the random seed once before calling generate_wgn()
	_send_obstacle_distance_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": obst_dist");
}

Sih::~Sih()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_send_obstacle_distance_perf);
}

void Sih::run()
{
	_px4_accel.set_temperature(T1_C);
	_px4_gyro.set_temperature(T1_C);

	parameters_updated();

	const hrt_abstime task_start = hrt_absolute_time();
	_last_run = task_start;
	_airspeed_time = task_start;
	_dist_snsr_time = task_start;
	_ranging_beacon_time = task_start;
	_vehicle = static_cast<VehicleType>(constrain(_sih_vtype.get(),
					    static_cast<int32_t>(VehicleType::First),
					    static_cast<int32_t>(VehicleType::Last)));

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	lockstep_loop();
#else
	realtime_loop();
#endif
	exit_and_cleanup(desc);
}

#if defined(ENABLE_LOCKSTEP_SCHEDULER)
// Get current timestamp in microseconds
static uint64_t micros()
{
	struct timeval t;
	gettimeofday(&t, nullptr);
	return t.tv_sec * ((uint64_t)1000000) + t.tv_usec;
}

void Sih::lockstep_loop()
{
	int rate = math::min(_imu_gyro_ratemax.get(), _imu_integration_rate.get());

	// default to 400Hz (2500 us interval)
	if (rate <= 0) {
		rate = 400;
	}

	// 200 - 2000 Hz
	int sim_interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);

	float speed_factor = 1.f;
	const char *speedup = getenv("PX4_SIM_SPEED_FACTOR");

	if (speedup) {
		speed_factor = atof(speedup);
	}

	int rt_interval_us = int(roundf(sim_interval_us / speed_factor));

	PX4_INFO("Simulation loop with %d Hz (%d us sim time interval)", rate, sim_interval_us);
	PX4_INFO("Simulation with %.1fx speedup. Loop with (%d us wall time interval)", (double)speed_factor, rt_interval_us);
	uint64_t pre_compute_wall_time_us;

	while (!should_exit()) {
		pre_compute_wall_time_us = micros();
		perf_count(_loop_interval_perf);

		_current_simulation_time_us += sim_interval_us;
		struct timespec ts;
		abstime_to_ts(&ts, _current_simulation_time_us);
		px4_clock_settime(CLOCK_MONOTONIC, &ts);

		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);

		// Only do lock-step once we received the first actuator output
		int sleep_time;
		uint64_t current_wall_time_us;

		if (_last_actuator_output_time <= 0) {
			PX4_DEBUG("SIH starting up - no lockstep yet");
			current_wall_time_us = micros();
			sleep_time = math::max(0, sim_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));

		} else {
			px4_lockstep_wait_for_components();
			current_wall_time_us = micros();
			sleep_time = math::max(0, rt_interval_us - (int)(current_wall_time_us - pre_compute_wall_time_us));
		}

		_achieved_speedup = 0.99f * _achieved_speedup + 0.01f * ((float)sim_interval_us / (float)(
					    current_wall_time_us - pre_compute_wall_time_us + sleep_time));
		usleep(sleep_time);
	}
}
#endif

static void timer_callback(void *sem)
{
	px4_sem_post((px4_sem_t *)sem);
}

void Sih::realtime_loop()
{
	int rate = _imu_gyro_ratemax.get();

	// default to 250 Hz (4000 us interval)
	if (rate <= 0) {
		rate = 250;
	}

	// 200 - 2000 Hz
	int interval_us = math::constrain(int(roundf(1e6f / rate)), 500, 5000);

	px4_sem_init(&_data_semaphore, 0, 0);
	hrt_call_every(&_timer_call, interval_us, interval_us, timer_callback, &_data_semaphore);

	while (!should_exit()) {
		px4_sem_wait(&_data_semaphore);     // periodic real time wakeup
		perf_begin(_loop_perf);
		sensor_step();
		perf_end(_loop_perf);
	}

	hrt_cancel(&_timer_call);
	px4_sem_destroy(&_data_semaphore);
}

void Sih::sensor_step()
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
		parameters_updated();
	}

	// Re-spawn the simulated vehicle on the armed->disarmed edge so the next
	// bench run starts from an identical clean initial condition. Resetting on
	// disarm (rather than arm) leaves the whole disarmed window for the
	// estimator to re-converge around the snapped-back pose before takeoff.
	vehicle_control_mode_s control_mode;

	if (_vehicle_control_mode_sub.update(&control_mode)) {
		if (_was_armed && !control_mode.flag_armed) {
			reset_vehicle_state();
		}

		_was_armed = control_mode.flag_armed;
	}

	perf_begin(_loop_perf);

	const hrt_abstime now = hrt_absolute_time();
	const float dt = (now - _last_run) * 1e-6f;
	_last_run = now;

	read_motors(dt);

	generate_force_and_torques(dt);

	equations_of_motion(dt);

	reconstruct_sensors_signals(now);

	if ((_vehicle == VehicleType::FixedWing
	     || _vehicle == VehicleType::TailsitterVTOL
	     || _vehicle == VehicleType::StandardVTOL)
	    && now - _airspeed_time >= 50_ms) {
		_airspeed_time = now;
		send_airspeed(now);
	}

	// distance sensor published at 50 Hz
	if (now - _dist_snsr_time >= 20_ms
	    && fabs(_distance_snsr_override) < 10000) {
		_dist_snsr_time = now;
		send_dist_snsr(now);
	}

	// obstacle ring published at 10 Hz
	if (now - _obst_distance_time >= 100_ms) {
		_obst_distance_time = now;
		send_obstacle_distance(now);
	}

	// ranging beacon published at 2 Hz (each beacon at 0.5 Hz)
	if (now - _ranging_beacon_time >= 500_ms) {
		_ranging_beacon_time = now;
		send_ranging_beacon(now);
	}

	publish_ground_truth(now);

	perf_end(_loop_perf);
}

void Sih::parameters_updated()
{
	_T_MAX = _sih_t_max.get();
	_Q_MAX = _sih_q_max.get();
	_L_ROLL = _sih_l_roll.get();
	_L_PITCH = _sih_l_pitch.get();
	_KDV = _sih_kdv.get();
	_KDW = _sih_kdw.get();
	_F_T_MAX = _sih_f_thrust_max.get();
	_F_Q_MAX = _sih_f_torque_max.get();

	// update the thruster models
	for (size_t i = 0; i < NUM_DYN_THRUSTER; i++) {
		if (_sih_f_ct0.get() > 0.0f && _sih_f_cp0.get() > 0.0f) {
			_thruster[i] = Thruster(_sih_forward_diameter_inch.get(), _sih_forward_rpm_max.get(),
						_sih_f_ct0.get(), _sih_f_ct1.get(), _sih_f_ct2.get(),
						_sih_f_cp0.get(), _sih_f_cp1.get(), _sih_f_cp2.get());

		} else {
			_thruster[i] = Thruster(_F_T_MAX, _F_Q_MAX);
		}
	}

	if (_sih_f_ct0.get() > 0.0f && _sih_f_cp0.get() > 0.0f) {
		_F_T_MAX = _thruster[0].get_T_max();
		_F_Q_MAX = _thruster[0].get_Q_max();

		if (fabsf(_F_T_MAX - _sih_f_thrust_max.get()) > 1.0e-5f) {
			_sih_f_thrust_max.set(_F_T_MAX);
			_sih_f_thrust_max.commit();
			PX4_INFO("SIH_F_CT0 > 0, using propeller dynamic model, overriding SIH_F_T_MAX");
		}

		if (fabsf(_F_Q_MAX - _sih_f_torque_max.get()) > 1.0e-5f) {
			_sih_f_torque_max.set(_F_Q_MAX);
			_sih_f_torque_max.commit();
			PX4_INFO("SIH_F_CP0 > 0, using propeller dynamic model, overriding SIH_F_Q_MAX");
		}
	}

	if (!_lpos_ref.isInitialized()
	    || (fabsf(static_cast<float>(_lpos_ref.getProjectionReferenceLat()) - _sih_lat0.get()) > FLT_EPSILON)
	    || (fabsf(static_cast<float>(_lpos_ref.getProjectionReferenceLon()) - _sih_lon0.get()) > FLT_EPSILON)
	    || (fabsf(_lpos_ref_alt - _sih_h0.get()) > FLT_EPSILON)) {
		_lpos_ref.initReference(static_cast<double>(_sih_lat0.get()), static_cast<double>(_sih_lon0.get()));
		_lpos_ref_alt = _sih_h0.get();

		reset_vehicle_state();
	}

	_MASS = _sih_mass.get();

	_I = diag(Vector3f(_sih_ixx.get(), _sih_iyy.get(), _sih_izz.get()));
	_I(0, 1) = _I(1, 0) = _sih_ixy.get();
	_I(0, 2) = _I(2, 0) = _sih_ixz.get();
	_I(1, 2) = _I(2, 1) = _sih_iyz.get();

	// guards against too small determinants
	_Im1 = 100.0f * inv(static_cast<typeof _I>(100.0f * _I));

	_distance_snsr_min = _sih_distance_snsr_min.get();
	_distance_snsr_max = _sih_distance_snsr_max.get();
	_distance_snsr_override = _sih_distance_snsr_override.get();

	_T_TAU = _sih_thrust_tau.get();

	_v_wind_N = Vector3f(_sih_wind_n.get(), _sih_wind_e.get(), 0.f);

	// SIH_TERR_EN selects one of four mutually exclusive ground modes:
	//   0 = off    : flat ground, no walls
	//   1 = terrain: fBm hills via terrain(N, E); no walls
	//   2 = walls  : flat ground + procedural wall lattice; fBm off
	//   3 = map    : real elevation from /fs/microsd/etc/terrain.pxtm
	// terrain() returns fBm only when mode == 1 (amp forced to 0
	// otherwise so the heightfield reads as flat). The wall lattice is
	// materialised into the SDF scene only when mode == 2. The DEM grid
	// is installed only in mode 3, and because lib/terrain composes grid
	// and fBm additively, mode 3 with SIH_TERR_AMP > 0 would layer
	// procedural detail over the real landform — not wired to a param
	// yet, keep amp at 0 for a faithful DEM.
	//
	// SIH_TERR_PLANE is independent of SIH_TERR_EN: a non-zero slope
	// angle engages planar mode in any ground mode, so the sloped-
	// landing acceptance test can run without also enabling walls or
	// hills. SIH's local frame is anchored at home (SIH_LOC_LAT0/LON0),
	// so home_n/home_e are both zero in the (N, E) coordinates passed
	// to terrain().
	const int32_t terr_mode = _sih_terr_en.get();
	const float terr_amp = (terr_mode == 1) ? _sih_terr_amp.get() : 0.f;
	const float terr_wavelength = 1.f / fmaxf(_sih_terr_freq.get(), 1e-6f);
	terrain_set_params(terr_amp, terr_wavelength, _sih_terr_seed.get(),
			   0.f, 0.f, _sih_terr_plane.get());

	// Heightmap: load on entry to mode 3, drop on any exit from it.
	// terrain_set_params() above already cleared the grid's contribution
	// from the home offset, so the load below recomputes it.
	//
	// The map file carries its own WGS84 origin in double precision
	// because SIH_LOC_LAT0/LON0 are ParamFloat and float32 only resolves
	// ~0.6 m of latitude. Projecting the file's origin through _lpos_ref
	// puts the DEM at its true offset from the local frame instead of
	// pinning its corner to local (0, 0).
	if (terr_mode == 3) {
		if (!_terrain_map.loaded()
		    && _terrain_map.load(TERRAIN_MAP_DEFAULT_PATH, 0.f, 0.f)) {
			// The header's origin is only readable after the load, so
			// anchor in a second step rather than re-reading the file.
			const matrix::Vector2f corner = _lpos_ref.project(
								_terrain_map.origin_lat(), _terrain_map.origin_lon());
			_terrain_map.set_origin(corner(0), corner(1));
		}

	} else if (_terrain_map.loaded()) {
		_terrain_map.unload();
	}

	// Procedural wall lattice. Walls are not materialized into a list;
	// scene_eval() queries them lazily per-cell via scene_wall_at_cell()
	// (a deterministic function of the SIH_TERR_SEED hash, like terrain()),
	// touching only the cells local to each ray sample. Walls and fBm are
	// mutually exclusive: only mode 2 enables walls. PX4 and the companion
	// viewer compile the same source so they agree on layouts. The seed is
	// read live, so runtime SIH_* param changes take effect immediately.
	sdf_scene_clear();
	sdf_walls_set_enabled(terr_mode == 2);

	// Corridor walls. These are solid boxes in the same scene list the
	// generic primitives use, so they compose with terrain by min()-union
	// rather than replacing it: the vehicle can fly out of a doorway and
	// land on open ground. sdf_scene_clear() above already emptied the
	// list, so this is an unconditional (re)install.
	//
	// A building that fails its own self-check is worse than no building:
	// every ring bin and every rangefinder would report a plausible wrong
	// distance with no other symptom. So a failed check leaves the scene
	// empty and the ground flat, which is visibly wrong rather than
	// quietly wrong.
	if (terr_mode == 4 && !corridor_demo_install_and_check()) {
		PX4_ERR("corridor: self-check failed, falling back to flat ground");
	}
}

void Sih::reset_vehicle_state()
{
	// Re-spawn the vehicle at a fixed, repeatable initial condition. The px4
	// process persists across arm/disarm cycles on hardware (unlike SITL, where
	// the test harness restarts the whole process per scenario), so without an
	// explicit reset the physics state integrates continuously and each bench
	// run inherits the previous run's pose -- attitude and position drift
	// run-to-run even though the board never moved, and the test is not
	// repeatable. Called at boot (parameters_updated) and on every disarm edge.

	// Zero all kinematics: at rest, level, heading North.
	_w_B = matrix::Vector3f{};
	_v_B = matrix::Vector3f{};
	_v_N = matrix::Vector3f{};
	_v_N_dot = matrix::Vector3f{};
	_v_E = matrix::Vector3f{};
	_v_E_dot = matrix::Vector3f{};
	_q = matrix::Quatf{};   // identity rotation: level, yaw 0

	// Spawn the vehicle so the landing gear rests AT terrain, not the CoM.
	// Without this offset the 4-gear contact model would see SIH_GEAR_Z of
	// penetration at boot, and the spring transient would tilt the airframe
	// enough to trip PX4's pre-arm "Attitude failure (pitch)" check. FW/Rover
	// keep the CoM-only hard-stop and spawn at the reference altitude exactly.
	_lla.setLatitudeDeg(static_cast<double>(_sih_lat0.get()));
	_lla.setLongitudeDeg(static_cast<double>(_sih_lon0.get()));
	const bool multirotor_family = (_vehicle == VehicleType::Quadcopter
					|| _vehicle == VehicleType::Hexacopter
					|| _vehicle == VehicleType::TailsitterVTOL
					|| _vehicle == VehicleType::StandardVTOL);
	const float spawn_alt = multirotor_family
				? (_lpos_ref_alt + _sih_gear_z.get())
				: _lpos_ref_alt;
	_lla.setAltitude(spawn_alt);
	_p_E = _lla.toEcef();

	const Dcmf R_E2N = _lla.computeRotEcefToNed();
	_R_N2E = R_E2N.transpose();
	_v_E = _R_N2E * _v_N;

	_q_E = Quatf(_R_N2E) * _q;
	_q_E.normalize();

	// Keep the published local position consistent with the re-spawn.
	_lpos_ref.project(_lla.latitude_deg(), _lla.longitude_deg(), _lpos(0), _lpos(1));
	_lpos(2) = -(_lla.altitude() - _lpos_ref_alt);
}

void Sih::read_motors(const float dt)
{
	actuator_outputs_s actuators_out;

	if (_actuator_out_sub.update(&actuators_out)) {
		_last_actuator_output_time = actuators_out.timestamp;

		for (int i = 0; i < NUM_ACTUATORS_MAX; i++) { // saturate the motor signals
			if ((_vehicle == VehicleType::FixedWing && i < 3) || (_vehicle == VehicleType::TailsitterVTOL && i > 3)) {
				_u[i] = actuators_out.output[i];

			} else {
				float u_sp = actuators_out.output[i];
				_u[i] = _u[i] + dt / _T_TAU * (u_sp - _u[i]); // first order transfer function with time constant tau
			}
		}
	}
}

void Sih::generate_force_and_torques(const float dt)
{
	// air-relative velocity in body frame [m/s]
	_v_B = _q_E.rotateVectorInverse(_R_N2E * _v_apparent_N);

	if (_vehicle == VehicleType::Quadcopter) {

		_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));
		_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]),
				 _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),
				 _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));

		_Fa_E = -_KDV * _R_N2E * _v_apparent_N; // first order drag to slow down the aircraft
		_Ma_B = -_KDW * _w_B; // first order angular damper

	} else if (_vehicle == VehicleType::Hexacopter) {
		/*     m5    m0      ┬
		         \  /      √3/2
		    m4 -- + -- m1    ┴
		         /  \
		       m3    m2
		          ├1/2┤
		          ├  1  ┤    */
		float u_sq[6];

		for (int i = 0; i < 6; ++i) {
			u_sq[i] = _u[i] * _u[i]; // quadratic thrust model, keep _u[i] intact for the filter
		}

		_T_B = Vector3f(0.0f, 0.0f, -_T_MAX * (+u_sq[0] + u_sq[1] + u_sq[2] + u_sq[3] + u_sq[4] + u_sq[5]));
		_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-.5f * u_sq[0] - u_sq[1] - .5f * u_sq[2] + .5f * u_sq[3] + u_sq[4] + .5f * u_sq[5]),
				 _L_PITCH * _T_MAX * (M_SQRT3_F / 2.f) * (+u_sq[0] - u_sq[2] - u_sq[3] + u_sq[5]),
				 _Q_MAX * (+u_sq[0] - u_sq[1] + u_sq[2] - u_sq[3] + u_sq[4] - u_sq[5]));
		_Fa_E = -_KDV * _R_N2E * _v_apparent_N; // first order drag to slow down the aircraft
		_Ma_B = -_KDW * _w_B; // first order angular damper

	} else if (_vehicle == VehicleType::FixedWing) {

		_T[0] = _thruster[0].compute_thrust_from_throttle(_u[3], _v_B(0));
		_Q[0] = _thruster[0].compute_torque_from_throttle(_u[3], _v_B(0));
		_T_B = Vector3f(_T[0], 0.0f, 0.0f); 	// forward thruster
		_Mt_B = Vector3f(_Q[0], 0.0f, 0.0f);	// thruster torque
		generate_fw_aerodynamics(_u[0], _u[1], _u[2], _T[0]);

	} else if (_vehicle == VehicleType::TailsitterVTOL) {

		for (size_t i = 0; i < NUM_DYN_THRUSTER; i++) {
			_T[i] = _thruster[i].compute_thrust_from_throttle(_u[i], -_v_B(2));
			_Q[i] = _thruster[i].compute_torque_from_throttle(_u[i], -_v_B(2));
		}

		_T_B = Vector3f(0.0f, 0.0f, -_T[0] - _T[1]);
		_Mt_B = Vector3f(_L_ROLL * (_T[1] - _T[0]), 0.0f, _Q[1] - _Q[0]);
		generate_ts_aerodynamics();

	} else if (_vehicle == VehicleType::StandardVTOL) {

		_T[0] = _thruster[0].compute_thrust_from_throttle(_u[7], _v_B(0));
		_Q[0] = _thruster[0].compute_torque_from_throttle(_u[7], _v_B(0));
		_T_B = Vector3f(_T[0], 0.0f, -_T_MAX * (+_u[0] + _u[1] + _u[2] + _u[3]));
		_Mt_B = Vector3f(_L_ROLL * _T_MAX * (-_u[0] + _u[1] + _u[2] - _u[3]) + _Q[0],
				 _L_PITCH * _T_MAX * (+_u[0] - _u[1] + _u[2] - _u[3]),
				 _Q_MAX * (+_u[0] + _u[1] - _u[2] - _u[3]));

		// thrust 0 means no propwash on the tail
		generate_fw_aerodynamics(_u[4], _u[5], _u[6], 0);

	} else if (_vehicle == VehicleType::RoverAckermann) {
		generate_rover_ackermann_dynamics(_u[1], _u[0], dt);
	}
}

void Sih::generate_fw_aerodynamics(const float roll_cmd, const float pitch_cmd, const float yaw_cmd,
				   const float thrust_for_prowash)
{
	const float &alt = _lla.altitude();

	_wing_l.update_aero(_v_B, _w_B, alt, roll_cmd * FLAP_MAX);
	_wing_r.update_aero(_v_B, _w_B, alt, -roll_cmd * FLAP_MAX);

	_tailplane.update_aero(_v_B, _w_B, alt, -pitch_cmd * FLAP_MAX, thrust_for_prowash);
	_fin.update_aero(_v_B, _w_B, alt, yaw_cmd * FLAP_MAX, thrust_for_prowash);
	_fuselage.update_aero(_v_B, _w_B, alt);

	// sum of aerodynamic forces
	const Vector3f Fa_B = _wing_l.get_Fa() + _wing_r.get_Fa() + _tailplane.get_Fa() + _fin.get_Fa() + _fuselage.get_Fa() -
			      _KDV * _v_B;
	_Fa_E = _q_E.rotateVector(Fa_B);

	// aerodynamic moments
	_Ma_B = _wing_l.get_Ma() + _wing_r.get_Ma() + _tailplane.get_Ma() + _fin.get_Ma() + _fuselage.get_Ma() - _KDW * _w_B;
}

void Sih::generate_ts_aerodynamics()
{
	// the aerodynamic is resolved in a frame like a standard aircraft (nose-right-belly)
	Vector3f v_ts = _R_S2B.transpose() * _v_B;
	Vector3f w_ts = _R_S2B.transpose() * _w_B;
	float altitude = _lpos_ref_alt - _lpos(2);

	Vector3f Fa_ts{};
	Vector3f Ma_ts{};

	for (int i = 0; i < NB_TS_SEG; i++) {
		if (i <= NB_TS_SEG / 2) {
			_ts[i].update_aero(v_ts, w_ts, altitude, _u[5]*TS_DEF_MAX, _T[1]);

		} else {
			_ts[i].update_aero(v_ts, w_ts, altitude, -_u[4]*TS_DEF_MAX, _T[0]);
		}

		Fa_ts += _ts[i].get_Fa();
		Ma_ts += _ts[i].get_Ma();
	}

	const Vector3f Fa_B = _R_S2B * Fa_ts - _KDV * _v_B; 	// sum of aerodynamic forces
	_Fa_E = _q_E.rotateVector(Fa_B);
	_Ma_B = _R_S2B * Ma_ts - _KDW * _w_B; 	// aerodynamic moments
}

void Sih::generate_rover_ackermann_dynamics(const float throttle_cmd, const float steering_cmd, const float dt)
{
	// --- Constants ---
	static constexpr float MAX_THROTTLE_FORCE = 400.0f;     // [N]
	static constexpr float MAX_STEER_ANGLE = radians(30.f); // [rad]
	static constexpr float WHEEL_BASE = 0.321f;             // [m] Distance between front and rear axle
	static constexpr float C = 500.f;                       // [N/rad] Cornering stiffness
	static constexpr float MU_S = 0.5f;                     // [-] Static Coefficient of friction
	static constexpr float MU_K = 0.4f;                     // [-] Kinetic Coefficient of friction
	static constexpr float MU_R = 0.3f;                     // [-] Rolling Coefficient of friction
	static constexpr float ROLLING_THRESHOLD = 0.05f;       // [m/s] Threshold for rolling resistance
	static constexpr float STATIC_THRESHOLD = 0.01f;        // [m/s] Threshold for static resistance

	matrix::Vector3f v_B = _q.rotateVectorInverse(_v_N); // Nav -> Body

	// --- Compute inputs ---
	const float delta = MAX_STEER_ANGLE * steering_cmd; // [rad] Steering angle
	const float F_x = MAX_THROTTLE_FORCE * throttle_cmd;         // [N] Throttle force

	// --- Compute forces and moments ---
	float F_y = 0.f; // [N] Lateral force
	float M_z = 0.f; // [Nm] Yaw moment

	if (fabsf(v_B(0)) > ROLLING_THRESHOLD) {
		// Equations based on the lateral dynamics of the bicycle model from [1]
		// [1] Sri Anumakonda, Everything you need to know about Self-Driving Cars in <30 minutes
		// Link: https://srianumakonda.medium.com/everything-you-need-to-know-about-self-driving-in-30-minutes-b38d68bd3427
		const float a_y = C * delta / _MASS - fabsf(v_B(0)) * _w_B(2)
				  - 2 * C * v_B(1) / (_MASS * fabsf(v_B(0))); // [m/s^2] Lateral acceleration
		const float psi_dot_dot = WHEEL_BASE * C * delta / _sih_izz.get()
					  - C * WHEEL_BASE * WHEEL_BASE  * _w_B(2) / (_sih_izz.get() * fabsf(v_B(0))); // [rad/s^2] Yaw acceleration
		F_y = _MASS * a_y; // Lateral force [N]
		M_z = _sih_izz.get() * psi_dot_dot; // [Nm] Yaw moment
	}

	_T_B = Vector3f(F_x, F_y, 0.f);
	_Mt_B = Vector3f(0.f, 0.f, M_z);

	// --- Compute drag/friction forces and moments ---
	Vector3f F_f = Vector3f(0.f, 0.f, 0.f); // [N] Friction force
	Vector3f F_a = Vector3f(0.f, 0.f, 0.f); // [N] Aerodynamic force (neglect until rover is rolling)

	if (_v_E.norm() < STATIC_THRESHOLD) { // Static friction
		Vector3f F_f_B = Vector3f(sign(F_x) * math::min(fabsf(F_x), MU_S * _MASS * 9.81f), sign(F_y) * math::min(fabsf(F_y),
					  MU_S * _MASS * 9.81f), 0.f);
		F_f = _q_E.rotateVector(F_f_B);

	} else if (_v_E.norm() < ROLLING_THRESHOLD) { // Kinetic friction
		if (_T_B.norm() > FLT_EPSILON) {
			F_f = _v_E.unit_or_zero() * MU_K * _MASS * 9.81f;

		} else {
			F_f = _v_E * _MASS / dt; // Stop the vehicle
		}

	} else { // Rolling friction
		F_f = _v_E.unit_or_zero() * MU_R * _MASS * 9.81f;
		Vector3f v_E_squared = Vector3f(sign(_v_E(0)) * _v_E(0) * _v_E(0), sign(_v_E(1)) * _v_E(1) * _v_E(1),
						sign(_v_E(2)) * _v_E(2) * _v_E(2));
		F_a = _KDV * v_E_squared; // [N] Second order drag
	}

	_Fa_E = -F_a - F_f;   // [N] Second order drag and friction
	_Ma_B = -_KDW * _w_B; // [Nm] First order angular damper

}

float Sih::computeGravity(const double lat)
{
	// Somigliana formula for gravitational acceleration
	const double sin_lat = sin(lat);
	const double g = LatLonAlt::Wgs84::gravity_equator * (1.0 + 0.001931851353 * sin_lat * sin_lat) / sqrt(
				 1.0 - LatLonAlt::Wgs84::eccentricity2 * sin_lat * sin_lat);
	return static_cast<float>(g);
}

void Sih::equations_of_motion(const float dt)
{
	const Vector3f gravity_acceleration_E = Vector3f(_R_N2E.col(2)) * computeGravity(
			_lla.latitude_rad()); // gravity along the Down axis
	const Vector3f coriolis_acceleration_E = -2.f * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE).cross(_v_E);

	const Vector3f weight_E = _MASS * gravity_acceleration_E;
	Vector3f sum_of_forces_E = _Fa_E + _q_E.rotateVector(_T_B) + weight_E;

	// fake ground, avoid free fall
	const float force_down = Vector3f(_R_N2E.transpose() * sum_of_forces_E)(2);
	Vector3f ground_force_E;
	_Mc_B = Vector3f();

	if (_vehicle == VehicleType::Quadcopter
	    || _vehicle == VehicleType::Hexacopter
	    || _vehicle == VehicleType::TailsitterVTOL
	    || _vehicle == VehicleType::StandardVTOL) {

		// 4-point landing-gear contact. Gears at the corners of the
		// (SIH_L_PITCH, SIH_L_ROLL) rectangle, offset downward by SIH_GEAR_Z
		// in body frame. Each gear runs the penetration / spring-damper
		// / Coulomb-friction physics independently and self-skips when above
		// terrain; per-gear forces sum into the body force, and per-gear
		// (r_gear x F_gear) sums into the body-frame contact torque _Mc_B,
		// so the vehicle tips on a slope and pivots about a single loaded gear.
		static constexpr int NUM_GEARS = 4;
		const float gear_z = _sih_gear_z.get();
		const Vector3f gears_body[NUM_GEARS] = {
			Vector3f(_L_PITCH,  _L_ROLL,  gear_z),
			Vector3f(_L_PITCH,  -_L_ROLL, gear_z),
			Vector3f(-_L_PITCH, -_L_ROLL, gear_z),
			Vector3f(-_L_PITCH, _L_ROLL,  gear_z),
		};

		Vector3f F_total_N(0.f, 0.f, 0.f);
		Vector3f M_total_B(0.f, 0.f, 0.f);
		bool any_grounded = false;

		// Per-gear spring-damper constants for the 4-point contact model.
		//   F_normal_per_gear = max(0, K * pen - C * v_normal)
		// then clamped to 10*M*g per gear so a boot-time penetration
		// transient can't launch the vehicle. Defaults (K=1000, C=32)
		// are tuned for a 1 kg multirotor with critical damping:
		//   C_critical = 2 * sqrt(K * M / 4)   (4 gears in parallel)
		// The Phase-3 originals (K=12500, C=1000) gave ~9x over-critical
		// damping and saturated the F-cap on every flat-ground touchdown,
		// producing a relaxation oscillator that prevented land_detector
		// from declaring landed. Tune in lockstep when changing K.
		const float k = _sih_ground_k.get();
		const float c = _sih_ground_c.get();
		const float mu = _sih_ground_mu.get();
		const float g_local = gravity_acceleration_E.norm();
		const Vector3f omega_N = _q.rotateVector(_w_B);

		for (int i = 0; i < NUM_GEARS; ++i) {
			const Vector3f r_body = gears_body[i];
			const Vector3f r_world = _q.rotateVector(r_body);
			const Vector3f gear_N = _lpos + r_world;
			const float gear_terrain_h = terrain(gear_N(0), gear_N(1));
			const float gear_alt = -gear_N(2);
			const float penetration = gear_terrain_h - gear_alt;

			if (penetration <= 0.f) {
				continue;
			}

			any_grounded = true;

			float dn = 0.f;
			float de = 0.f;
			terrain_gradient(gear_N(0), gear_N(1), &dn, &de);
			Vector3f n_hat_N(-dn, -de, -1.f);
			n_hat_N.normalize();

			if (!PX4_ISFINITE(n_hat_N(0)) || !PX4_ISFINITE(n_hat_N(1)) || !PX4_ISFINITE(n_hat_N(2))) {
				n_hat_N = Vector3f(0.f, 0.f, -1.f);
			}

			// Gear tip velocity in NED: body translation plus omega x r_world.
			const Vector3f v_gear_N = _v_N + omega_N.cross(r_world);
			const float v_normal = v_gear_N.dot(n_hat_N);

			float F_normal_mag = math::max(0.f, k * penetration - c * v_normal);
			const float F_normal_cap = 10.f * _MASS * g_local;
			F_normal_mag = math::min(F_normal_mag, F_normal_cap);
			const Vector3f F_normal_N = F_normal_mag * n_hat_N;

			Vector3f F_friction_N(0.f, 0.f, 0.f);
			const Vector3f v_tangent_N = v_gear_N - v_normal * n_hat_N;
			const float v_tangent_norm = v_tangent_N.norm();

			if (v_tangent_norm > 1e-3f && F_normal_mag > 0.f) {
				const float F_friction_cap = mu * F_normal_mag;
				const float F_required_to_stop = _MASS * v_tangent_norm / dt;

				if (F_required_to_stop <= F_friction_cap) {
					F_friction_N = -(_MASS / dt) * v_tangent_N;

				} else {
					F_friction_N = -F_friction_cap * v_tangent_N / v_tangent_norm;
				}
			}

			const Vector3f F_gear_N = F_normal_N + F_friction_N;
			F_total_N += F_gear_N;

			// Torque about CoM: r_body x F_body. Convert F from NED to body.
			const Vector3f F_gear_B = _q.rotateVectorInverse(F_gear_N);
			M_total_B += r_body.cross(F_gear_B);
		}

		ground_force_E = _R_N2E * F_total_N;
		_Mc_B = M_total_B;
		_grounded = any_grounded;

	} else if (_vehicle == VehicleType::FixedWing
		   || _vehicle == VehicleType::RoverAckermann) {

		// FW/Rover keep the CoM-only hard-stop. Multi-point gear for these
		// vehicle types is out of scope for now.
		const float terrain_h = terrain(_lpos(0), _lpos(1));

		if ((_lla.altitude() - _lpos_ref_alt - terrain_h) < 0.f && force_down > 0.f) {
			Vector3f down_u = _R_N2E.col(2);
			ground_force_E = -down_u * sum_of_forces_E * down_u;

			if (!_grounded) {
				// if we just hit the floor
				// compute the force that will stop the vehicle in one time step
				ground_force_E += down_u * (-_v_N(2) / dt) * _MASS;
			}

			_grounded = true;

		} else {
			_grounded = false;
		}
	}

	sum_of_forces_E += ground_force_E;
	const Vector3f acceleration_E = sum_of_forces_E / _MASS;
	_specific_force_E = acceleration_E - gravity_acceleration_E;

	_v_E_dot = acceleration_E + coriolis_acceleration_E;

	// add fictitious transport rate acceleration as the local navigation frame rotates
	// to stay tangent to the ellipsoid
	const Vector3f transport_rate = -_lla.computeAngularRateNavFrame(_v_N).cross(_v_N);
	_v_N_dot = _R_N2E.transpose() * _v_E_dot + transport_rate;

	// forward Euler velocity intergation
	Vector3f v_E_prev = _v_E;
	_v_E = _v_E + _v_E_dot * dt;
	// trapezoidal position integration
	_p_E = _p_E + Vector3d(_v_E + v_E_prev) * 0.5 * static_cast<double>(dt);

	const Quatf dq(AxisAnglef(_w_B * dt));

	_q_E = _q_E  * dq;
	_q_E.normalize();

	const Vector3f w_B_dot = _Im1 * (_Mt_B + _Ma_B + _Mc_B - _w_B.cross(_I * _w_B)); // conservation of angular momentum
	_w_B = constrain(_w_B + w_B_dot * dt, -6.0f * M_PI_F, 6.0f * M_PI_F);

	ecefToNed();

	_lpos_ref.project(_lla.latitude_deg(), _lla.longitude_deg(), _lpos(0), _lpos(1));
	_lpos(2) = -(_lla.altitude() - _lpos_ref_alt);
}

void Sih::ecefToNed()
{
	_lla = LatLonAlt::fromEcef(_p_E);

	const Dcmf C_SE = _lla.computeRotEcefToNed();
	_R_N2E = C_SE.transpose();

	// Transform velocity to NED frame
	_v_N = C_SE * _v_E;
	_v_apparent_N = _v_N + _v_wind_N;

	_q = Quatf(C_SE) * _q_E;
	_q.normalize();
}

void Sih::reconstruct_sensors_signals(const hrt_abstime &time_now_us)
{
	// The sensor signals reconstruction and noise levels are from [1]
	// [1] Bulka, Eitan, and Meyer Nahon. "Autonomous fixed-wing aerobatics: from theory to flight."
	//     In 2018 IEEE International Conference on Robotics and Automation (ICRA), pp. 6573-6580. IEEE, 2018.

	// IMU
	const Dcmf R_E2B(_q_E.inversed());
	Vector3f accel_noise;
	Vector3f gyro_noise;

	if (_T_B.longerThan(FLT_EPSILON)) {
		accel_noise = noiseGauss3f(0.5f, 1.7f, 1.4f);
		gyro_noise = noiseGauss3f(0.14f, 0.07f, 0.03f);

	} else {
		// Lower noise when not armed
		accel_noise = noiseGauss3f(0.1f, 0.1f, 0.1f);
		gyro_noise = noiseGauss3f(0.01f, 0.01f, 0.01f);
	}

	Vector3f specific_force_B = R_E2B * _specific_force_E;
	Vector3f accel = specific_force_B + accel_noise;

	const Vector3f earth_spin_rate_B = R_E2B * Vector3f(0.f, 0.f, CONSTANTS_EARTH_SPIN_RATE);
	Vector3f gyro = _w_B + earth_spin_rate_B + gyro_noise;

	// update IMU every iteration
	_px4_accel.update(time_now_us, accel(0), accel(1), accel(2));
	_px4_gyro.update(time_now_us, gyro(0), gyro(1), gyro(2));
}

void Sih::send_airspeed(const hrt_abstime &time_now_us)
{
	// TODO: send differential pressure instead?
	airspeed_s airspeed{};
	airspeed.timestamp_sample = time_now_us;

	// Assume the pitot tube always points against the wind to not have tailsitter edge cases
	airspeed.true_airspeed_m_s = fmaxf(0.1f, _v_apparent_N.norm() + generate_wgn() * 0.2f);
	airspeed.indicated_airspeed_m_s = airspeed.true_airspeed_m_s * sqrtf(_wing_l.get_rho() / RHO);
	airspeed.confidence = 0.7f;
	airspeed.timestamp = hrt_absolute_time();
	_airspeed_pub.publish(airspeed);
}

void Sih::send_dist_snsr(const hrt_abstime &time_now_us)
{
	// One distance_sensor instance is published per bit set in
	// SIH_DISTSNSR_DIR. Bit 0 (downward) is the default; forward/left/
	// right/up are opt-in. SIH_DISTSNSR_MIN/MAX/OVR apply to every
	// enabled instance (see the yaml `long:` docs on each param).
	//
	// Each instance casts a beam against the simulated world. The downward
	// instance uses `lib/terrain`'s analytical fast-path (closed-form
	// intersection with the ground, near-vertical only). The
	// forward / left / right / up instances route through
	// `lib/terrain_sdf`'s sphere tracer, which handles arbitrary beam
	// directions against the ground plus the SDF scene of walls
	// populated in parameters_updated(). At SIH_TERR_EN=0 the SDF scene
	// is empty (no walls) and the ground is flat, so the
	// analytical downward formula reduces to `altitude / cos(tilt)` and
	// the default config (DIR=1, EN=0) stays byte-identical to the
	// legacy single-instance publish.
	struct OrientationEntry {
		uint8_t orientation;    // distance_sensor_s::ROTATION_*
		float   dir_body[3];    // unit beam direction in body FRD; Vector3f ctor is not constexpr
	};
	static constexpr OrientationEntry table[NUM_DISTSNSR_INSTANCES] = {
		{ distance_sensor_s::ROTATION_DOWNWARD_FACING, { 0.f,  0.f,  1.f} }, // bit 0
		{ distance_sensor_s::ROTATION_FORWARD_FACING,  { 1.f,  0.f,  0.f} }, // bit 1
		{ distance_sensor_s::ROTATION_LEFT_FACING,     { 0.f, -1.f,  0.f} }, // bit 2
		{ distance_sensor_s::ROTATION_RIGHT_FACING,    { 0.f,  1.f,  0.f} }, // bit 3
		{ distance_sensor_s::ROTATION_UPWARD_FACING,   { 0.f,  0.f, -1.f} }, // bit 4
	};

	const int32_t dir_mask = _sih_distance_snsr_dir.get();

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_SIMULATION;
	device_id.devid_s.bus = 0;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_SIM;

	const float origin_n = _lpos(0);
	const float origin_e = _lpos(1);
	const float origin_alt = -_lpos(2); // metres above home; terrain() returns height above home

	for (uint8_t i = 0; i < NUM_DISTSNSR_INSTANCES; i++) {
		if (!(dir_mask & (1 << i))) {
			continue;
		}

		const matrix::Vector3f dir_body(table[i].dir_body[0], table[i].dir_body[1], table[i].dir_body[2]);
		const matrix::Vector3f dir_world = _q.rotateVector(dir_body);

		distance_sensor_s distance_sensor{};
		device_id.devid_s.address = i;
		distance_sensor.device_id     = device_id.devid;
		distance_sensor.type          = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
		distance_sensor.orientation   = table[i].orientation;
		distance_sensor.min_distance  = _distance_snsr_min;
		distance_sensor.max_distance  = _distance_snsr_max;
		distance_sensor.signal_quality = -1;

		if (_distance_snsr_override >= 0.f) {
			distance_sensor.current_distance = _distance_snsr_override;

		} else {
			// Both raycast() and sdf_sphere_trace() take an altitude-up Z
			// direction, so flip the Z sign on the NED-frame `dir_world`
			// produced by `_q.rotateVector`.
			const sdf_vec3 sdf_origin = {origin_n, origin_e, origin_alt};
			const sdf_vec3 sdf_dir    = {dir_world(0), dir_world(1), -dir_world(2)};
			float t_hit;

			if (table[i].orientation == distance_sensor_s::ROTATION_DOWNWARD_FACING) {
				// Near-vertical fast-path: the closed-form in `lib/terrain`
				// is cheap and exact while the beam is within ~5 deg of
				// vertical (level hover / landing).
				t_hit = raycast(origin_n, origin_e, origin_alt,
						dir_world(0), dir_world(1), -dir_world(2),
						_distance_snsr_max);

				if (t_hit >= _distance_snsr_max) {
					// raycast() reports a miss both for a genuine miss and
					// when the beam tilts out of the analytical envelope
					// (pitched / banked cruise), where the closed-form is
					// invalid. Fall back to the sphere tracer, which resolves
					// the hit at any angle against the heightfield + scene, so
					// the downward rangefinder does not drop out whenever the
					// vehicle is not level.
					t_hit = sdf_sphere_trace(sdf_origin, sdf_dir, _distance_snsr_max);
				}

			} else {
				// Forward / left / right / up need the sphere tracer so they
				// see static scene primitives in addition to the heightfield.
				t_hit = sdf_sphere_trace(sdf_origin, sdf_dir, _distance_snsr_max);
			}

			if (t_hit < 0.f) {
				// Origin-inside-geometry sentinel per the `lib/terrain_sdf`
				// contract (every orientation can reach the tracer now). The
				// vehicle is clipping scene geometry; report the bin as
				// saturated-near (current_distance = 0) with quality = 0 so
				// consumers gate on the quality marker rather than acting on a
				// spurious zero.
				distance_sensor.current_distance = 0.f;
				distance_sensor.signal_quality   = 0;
				distance_sensor.variance         = 1.0f;
				distance_sensor.timestamp        = hrt_absolute_time();
				_distance_snsr_pubs[i].publish(distance_sensor);
				continue;
			}

			if (t_hit >= _distance_snsr_max) {
				// No measurement (beam exited range — e.g. banked cruise
				// tilting the world-frame beam above the horizon). Use the
				// MAVLink "max + 1" no-measurement marker plus signal_quality
				// = 0 so EKF2 / collision_prevention gate the sample via
				// their existing innovation / quality checks instead of
				// seeing a 655 m UINT16_MAX/100 spike. Variance set large
				// (1 m^2) as belt-and-braces for consumers that only read
				// variance and not signal_quality.
				distance_sensor.current_distance = _distance_snsr_max + 0.01f;
				distance_sensor.signal_quality = 0;
				distance_sensor.variance = 1.0f;

			} else {
				distance_sensor.current_distance = t_hit;

				// Signal quality drops near max range AND
				// at steep oblique angles. Oblique angle is the angle
				// between the incoming beam and the local terrain
				// normal. terrain_gradient() at the hit point gives
				// (dh/dN, dh/dE); the outward (up) normal in NED is
				// (-dn, -de, -1).normalize().
				const float hit_n = origin_n + dir_world(0) * t_hit;
				const float hit_e = origin_e + dir_world(1) * t_hit;
				float dn = 0.f;
				float de = 0.f;
				terrain_gradient(hit_n, hit_e, &dn, &de);
				matrix::Vector3f normal_up(-dn, -de, -1.f);
				normal_up.normalize();
				const float cos_oblique = math::constrain(fabsf(-dir_world.dot(normal_up)), 0.f, 1.f);
				const float oblique_deg = math::degrees(acosf(cos_oblique));

				float quality = 100.f
						- 50.f * (t_hit / _distance_snsr_max)
						- 100.f * (oblique_deg / 60.f);
				quality = math::constrain(quality, 0.f, 100.f);

				const float q_norm = 1.f - quality / 100.f;
				distance_sensor.signal_quality = static_cast<int8_t>(quality);
				distance_sensor.variance = q_norm * q_norm * 0.01f; // m^2, baseline 0.01
			}
		}

		distance_sensor.timestamp = hrt_absolute_time();
		_distance_snsr_pubs[i].publish(distance_sensor);
	}
}

void Sih::send_obstacle_distance(const hrt_abstime &time_now_us)
{
	perf_begin(_send_obstacle_distance_perf);

	if (!_sih_obst_en.get()) {
		perf_end(_send_obstacle_distance_perf);
		return;
	}

	// Produce-when-consumed gate. The obstacle ring is only read by
	// CollisionPrevention, which runs in manual position control (POSCTL)
	// and only while flying. Skipping it when disarmed/landed or in other
	// modes avoids synthesising a sensor stream nobody reads — and, since
	// the horizontal sphere trace is most expensive grazing the ground at
	// low altitude, it removes the worst-case cost in exactly the resting/
	// landed state where the ring is useless anyway.
	vehicle_control_mode_s control_mode{};
	_vehicle_control_mode_sub.copy(&control_mode);
	vehicle_land_detected_s land_detected{};
	_vehicle_land_detected_sub.copy(&land_detected);

	const bool cp_active = control_mode.flag_armed
			       && control_mode.flag_control_position_enabled
			       && control_mode.flag_control_manual_enabled;

	if (!cp_active || land_detected.landed) {
		perf_end(_send_obstacle_distance_perf);
		return;
	}

	const float max_t = _sih_obst_max.get();
	const float origin_n = _lpos(0);
	const float origin_e = _lpos(1);
	const float origin_alt = -_lpos(2);
	const uint16_t max_cm = static_cast<uint16_t>(max_t * 100.f);

	for (uint8_t i = 0; i < OBST_BINS_PER_CYCLE; i++) {
		const uint8_t bin = (_obst_sweep_offset + i) % 72;
		const float angle_rad = math::radians(bin * 5.f);

		// Body FRD, bin 0 = vehicle forward, sweep clockwise per MAVLink
		// OBSTACLE_DISTANCE convention. `_q` (body -> NED) rotates into
		// the NED Z-down direction; the sphere tracer's `sdf_vec3` uses
		// altitude-up, so flip Z at the boundary.
		const matrix::Vector3f body_dir(cosf(angle_rad), sinf(angle_rad), 0.f);
		const matrix::Vector3f world_dir = _q.rotateVector(body_dir);

		const sdf_vec3 sdf_origin = {origin_n, origin_e, origin_alt};
		const sdf_vec3 sdf_dir    = {world_dir(0), world_dir(1), -world_dir(2)};
		const float t = sdf_sphere_trace(sdf_origin, sdf_dir, max_t);

		if (t < 0.f) {
			// Origin-inside-geometry sentinel per the `lib/terrain_sdf`
			// contract. The vehicle is clipping scene geometry in this
			// bin's direction; report `current_distance = 0` so consumers
			// (collision_prevention) treat the bin as saturated-near.
			_obst_bin_cache[bin] = 0;

		} else if (t >= max_t) {
			// `max_distance + 1` is the "no obstacle in range" marker per
			// ObstacleDistance.msg; consumers treat it as "bin clear",
			// not "ignore".
			_obst_bin_cache[bin] = static_cast<uint16_t>(max_cm + 1);

		} else {
			_obst_bin_cache[bin] = static_cast<uint16_t>(t * 100.f);
		}
	}

	_obst_sweep_offset = (_obst_sweep_offset + OBST_BINS_PER_CYCLE) % 72;

	obstacle_distance_s msg{};
	msg.timestamp = time_now_us;
	msg.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	msg.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	msg.increment = 5.0f;
	msg.angle_offset = 0.0f;
	msg.min_distance = 1;
	msg.max_distance = max_cm;

	static_assert(sizeof(msg.distances) == 72 * sizeof(uint16_t),
		      "obstacle_distance.distances width changed upstream");
	memcpy(msg.distances, _obst_bin_cache, sizeof(msg.distances));

	_obstacle_distance_pub.publish(msg);

	perf_end(_send_obstacle_distance_perf);
}

void Sih::send_ranging_beacon(const hrt_abstime &time_now_us)
{
	if (_lpos_ref.isInitialized()) {

		if (!_beacons_configured) {
			_beacons_configured = true;

			for (uint8_t i = 0; i < NUM_RANGING_BEACONS; i++) {
				_lpos_ref.reproject(RANGING_BEACON_OFFSETS[i].north_m, RANGING_BEACON_OFFSETS[i].east_m,
						    _ranging_beacons[i].lat_deg, _ranging_beacons[i].lon_deg);
				_ranging_beacons[i].alt_m = _sih_h0.get() + RANGING_BEACON_OFFSETS[i].alt_offset_m;
			}
		}

		const RangingBeaconConfig &beacon = _ranging_beacons[_ranging_beacon_idx];
		const LatLonAlt beacon_lla(beacon.lat_deg, beacon.lon_deg, beacon.alt_m);
		const matrix::Vector3d beacon_ecef = beacon_lla.toEcef();

		// Compute true range in ECEF
		const matrix::Vector3d delta_ecef = beacon_ecef - _p_E;
		const double true_range_m = delta_ecef.norm();

		const float noise_std = _sih_ranging_beacon_noise.get();
		const float noise_m = (noise_std > 0.f) ? generate_wgn() * noise_std : 0.f;
		const double measured_range_m = math::max(0.0, true_range_m + static_cast<double>(noise_m));

		ranging_beacon_s msg{};
		msg.timestamp = hrt_absolute_time();
		msg.timestamp_sample = time_now_us;
		msg.beacon_id = _ranging_beacon_idx;
		msg.range = static_cast<float>(measured_range_m);
		msg.lat = beacon.lat_deg;
		msg.lon = beacon.lon_deg;
		msg.alt = beacon.alt_m;
		msg.alt_type = 0; // WGS84
		msg.hacc = 1.0f;
		msg.vacc = 1.0f;
		msg.range_accuracy = noise_std;
		msg.sequence_nr = 0;
		msg.status = 0;
		msg.carrier_freq = 0;

		_ranging_beacon_pub.publish(msg);

		// cycle through the beacons
		_ranging_beacon_idx = (_ranging_beacon_idx + 1) % NUM_RANGING_BEACONS;
	}
}

void Sih::publish_ground_truth(const hrt_abstime &time_now_us)
{
	{
		// publish angular velocity groundtruth
		vehicle_angular_velocity_s angular_velocity{};
		angular_velocity.timestamp_sample = time_now_us;
		angular_velocity.xyz[0] = _w_B(0); // rollspeed;
		angular_velocity.xyz[1] = _w_B(1); // pitchspeed;
		angular_velocity.xyz[2] = _w_B(2); // yawspeed;
		angular_velocity.timestamp = hrt_absolute_time();
		_angular_velocity_ground_truth_pub.publish(angular_velocity);
	}

	{
		// publish attitude groundtruth
		vehicle_attitude_s attitude{};
		attitude.timestamp_sample = time_now_us;
		_q.copyTo(attitude.q);
		attitude.timestamp = hrt_absolute_time();
		_attitude_ground_truth_pub.publish(attitude);
	}

	{
		// publish local position groundtruth
		vehicle_local_position_s local_position{};
		local_position.timestamp_sample = time_now_us;

		local_position.xy_valid = true;
		local_position.z_valid = true;
		local_position.v_xy_valid = true;
		local_position.v_z_valid = true;

		local_position.x = _lpos(0);
		local_position.y = _lpos(1);
		local_position.z = _lpos(2);

		local_position.vx = _v_N(0);
		local_position.vy = _v_N(1);
		local_position.vz = _v_N(2);

		local_position.z_deriv = _v_N(2);

		local_position.ax = _v_N_dot(0);
		local_position.ay = _v_N_dot(1);
		local_position.az = _v_N_dot(2);

		local_position.xy_global = true;
		local_position.z_global = true;
		local_position.ref_timestamp = _last_run;
		local_position.ref_lat = _lpos_ref.getProjectionReferenceLat();
		local_position.ref_lon = _lpos_ref.getProjectionReferenceLon();
		local_position.ref_alt = _lpos_ref_alt;

		local_position.heading = Eulerf(_q).psi();
		local_position.heading_good_for_control = true;
		local_position.unaided_heading = NAN;

		local_position.timestamp = hrt_absolute_time();
		_local_position_ground_truth_pub.publish(local_position);
	}

	{
		// publish global position groundtruth
		vehicle_global_position_s global_position{};
		global_position.timestamp_sample = time_now_us;
		global_position.lat = _lla.latitude_deg();
		global_position.lon = _lla.longitude_deg();
		global_position.alt = _lla.altitude();
		global_position.alt_ellipsoid = global_position.alt;
		global_position.terrain_alt = -_lpos(2);
		global_position.timestamp = hrt_absolute_time();
		_global_position_ground_truth_pub.publish(global_position);
	}
}

float Sih::generate_wgn()   // generate white Gaussian noise sample with std=1
{
	// algorithm 1:
	// float temp=((float)(rand()+1))/(((float)RAND_MAX+1.0f));
	// return sqrtf(-2.0f*logf(temp))*cosf(2.0f*M_PI_F*rand()/RAND_MAX);
	// algorithm 2: from BlockRandGauss.hpp
	static float V1, V2, S;
	static bool phase = true;
	float X;

	if (phase) {
		do {
			float U1 = (float)rand() / (float)RAND_MAX;
			float U2 = (float)rand() / (float)RAND_MAX;
			V1 = 2.0f * U1 - 1.0f;
			V2 = 2.0f * U2 - 1.0f;
			S = V1 * V1 + V2 * V2;
		} while (S >= 1.0f || fabsf(S) < 1e-8f);

		X = V1 * float(sqrtf(-2.0f * float(logf(S)) / S));

	} else {
		X = V2 * float(sqrtf(-2.0f * float(logf(S)) / S));
	}

	phase = !phase;
	return X;
}

Vector3f Sih::noiseGauss3f(float stdx, float stdy, float stdz)
{
	return Vector3f(generate_wgn() * stdx, generate_wgn() * stdy, generate_wgn() * stdz);
}

int Sih::print_status()
{
#if defined(ENABLE_LOCKSTEP_SCHEDULER)
	PX4_INFO("Running in lockstep mode");
	PX4_INFO("Achieved speedup: %.2fX", (double)_achieved_speedup);
#endif

	if (_vehicle == VehicleType::Quadcopter) {
		PX4_INFO("Quadcopter");

	} else if (_vehicle == VehicleType::Hexacopter) {
		PX4_INFO("Hexacopter");

	} else if (_vehicle == VehicleType::FixedWing) {
		PX4_INFO("Fixed-Wing");
		PX4_INFO("propeller model:");
		_thruster[0].print_status();

	} else if (_vehicle == VehicleType::TailsitterVTOL) {
		PX4_INFO("TailSitter");
		PX4_INFO("propeller model:");
		_thruster[0].print_status();
		PX4_INFO("aoa [deg]: %d", (int)(degrees(_ts[4].get_aoa())));
		PX4_INFO("v segment (m/s)");
		_ts[4].get_vS().print();

	} else if (_vehicle == VehicleType::StandardVTOL) {
		PX4_INFO("Standard VTOL");
		PX4_INFO("pusher propeller model:");
		_thruster[0].print_status();

	} else if (_vehicle == VehicleType::RoverAckermann) {
		PX4_INFO("Rover Ackermann");
	}

	PX4_INFO("vehicle landed: %d", _grounded);
	PX4_INFO("local position NED (m)");
	_lpos.print();
	PX4_INFO("local velocity NED (m/s)");
	_v_N.print();
	PX4_INFO("attitude roll-pitch-yaw (deg)");
	(Eulerf(_q) * 180.0f / M_PI_F).print();
	PX4_INFO("angular acceleration roll-pitch-yaw (deg/s)");
	(_w_B * 180.0f / M_PI_F).print();
	PX4_INFO("actuator signals");
	Vector<float, 8> u = Vector<float, 8>(_u);
	u.transpose().print();
	PX4_INFO("Aerodynamic forces NED (N)");
	(_R_N2E.transpose() * _Fa_E).print();
	PX4_INFO("Aerodynamic moments body frame (Nm)");
	_Ma_B.print();
	PX4_INFO("Thruster forces in body frame (N)");
	_T_B.print();
	PX4_INFO("Thruster moments in body frame (Nm)");
	_Mt_B.print();
	return 0;
}

int Sih::run_trampoline(int argc, char *argv[])
{
	return ModuleBase::run_trampoline_impl(desc, [](int ac, char *av[]) -> ModuleBase * {
		return Sih::instantiate(ac, av);
	}, argc, argv);
}

int Sih::task_spawn(int argc, char *argv[])
{
	desc.task_id = px4_task_spawn_cmd("sih",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1560,
					  (px4_main_t)&run_trampoline,
					  (char *const *)argv);

	if (desc.task_id < 0) {
		desc.task_id = -1;
		return -errno;
	}

	return 0;
}

Sih *Sih::instantiate(int argc, char *argv[])
{
	Sih *instance = new Sih();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

int Sih::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int Sih::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a simulator for quadrotors and fixed-wings running fully
inside the hardware autopilot.

This simulator subscribes to "actuator_outputs" which are the actuator pwm
signals given by the control allocation module.

This simulator publishes the sensors signals corrupted with realistic noise
in order to incorporate the state estimator in the loop.

### Implementation
The simulator implements the equations of motion using matrix algebra.
Quaternion representation is used for the attitude.
Forward Euler is used for integration.
Most of the variables are declared global in the .hpp file to avoid stack overflow.


)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("simulator_sih", "simulation");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int simulator_sih_main(int argc, char *argv[])
{
	return ModuleBase::main(Sih::desc, argc, argv);
}
