#include "IEKF.hpp"
#include "matrix/filter.hpp"

static const float mag_inclination = 1.0f;
static const float mag_declination = 0;

IEKF::IEKF() :
	_nh(), // node handle
	_subImu(_nh.subscribe("sensor_combined", 0, &IEKF::callbackImu, this)),
	_subGps(_nh.subscribe("vehicle_gps_position", 0, &IEKF::correctGps, this)),
	_subAirspeed(_nh.subscribe("airspeed", 0, &IEKF::correctAirspeed, this)),
	_pubAttitude(_nh.advertise<vehicle_attitude_s>("vehicle_attitude", 0)),
	_pubLocalPosition(_nh.advertise<vehicle_local_position_s>("vehicle_local_position", 0)),
	_pubGlobalPosition(_nh.advertise<vehicle_global_position_s>("vehicle_global_position", 0)),
	_pubControlState(_nh.advertise<control_state_s>("control_state", 0)),
	_pubEstimatorStatus(_nh.advertise<estimator_status_s>("estimator_status", 0)),
	_x0(),
	_P0Diag(),
	_x(),
	_P(),
	_u(),
	_g_n(0, 0, -9.8),
	_B_n(),
	_origin(),
	_timestampAccel(),
	_timestampMag(),
	_timestampBaro(),
	_timestampGps()
{
	// initialize state
	_x0(X::q_nb_0) = 1;
	_x0(X::q_nb_1) = 0;
	_x0(X::q_nb_2) = 0;
	_x0(X::q_nb_3) = 0;
	_x0(X::accel_scale) = 1;
	_x = _x0;

	// initialize covariance
	_P0Diag(Xe::rot_n) = 1;
	_P0Diag(Xe::rot_e) = 1;
	_P0Diag(Xe::rot_d) = 3;
	_P0Diag(Xe::vel_n) = 1e9;
	_P0Diag(Xe::vel_e) = 1e9;
	_P0Diag(Xe::vel_d) = 1e9;
	_P0Diag(Xe::gyro_bias_n) = 1e-4;
	_P0Diag(Xe::gyro_bias_e) = 1e-4;
	_P0Diag(Xe::gyro_bias_d) = 1e-4;
	_P0Diag(Xe::accel_scale) = 1e-2;
	_P0Diag(Xe::pos_n) = 1e9;
	_P0Diag(Xe::pos_e) = 1e9;
	_P0Diag(Xe::pos_d) = 1e9;
	_P0Diag(Xe::terrain_alt) = 1e9;
	_P0Diag(Xe::baro_bias) = 1e9;
	_P0Diag(Xe::wind_n) = 1e9;
	_P0Diag(Xe::wind_e) = 1e9;
	_P0Diag(Xe::wind_d) = 1e9;
	_P = diag(_P0Diag);

	// initial magnetic field guess
	_B_n = Vector3f(0.21523, 0.00771, -0.42741);
}

Vector<float, X::n> IEKF::dynamics(float t, const Vector<float, X::n> &x, const Vector<float, U::n> &u)
{
	Quatf q_nb(x(X::q_nb_0), x(X::q_nb_1), x(X::q_nb_2), x(X::q_nb_3));
	Vector3f a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Vector3f as_n = a_n - _g_n;
	Vector3f gyro_bias_b(_x(X::gyro_bias_bx), _x(X::gyro_bias_by), _x(X::gyro_bias_bz));
	Vector3f omega_nb_b(_u(U::omega_nb_bx), _u(U::omega_nb_by), _u(U::omega_nb_bz));
	Vector3f omega_nb_b_corrected = omega_nb_b - gyro_bias_b;
	Quatf dq_nb = q_nb * Quatf(0, omega_nb_b_corrected(0),
				   omega_nb_b_corrected(1), omega_nb_b_corrected(2)) * 0.5f;
	//ROS_INFO("a_b: %10.4f %10.4f %10.4f\n", double(a_b(0)), double(a_b(1)), double(a_b(2)));
	//ROS_INFO("as_n: %10.4f %10.4f %10.4f\n", double(as_n(0)), double(as_n(1)), double(as_n(2)));

	Vector<float, X::n> dx;
	dx(X::q_nb_0) = dq_nb(0);
	dx(X::q_nb_1) = dq_nb(1);
	dx(X::q_nb_2) = dq_nb(2);
	dx(X::q_nb_3) = dq_nb(3);
	dx(X::vel_n) = as_n(0);
	dx(X::vel_e) = as_n(1);
	dx(X::vel_d) = as_n(2);
	dx(X::gyro_bias_bx) = 0;
	dx(X::gyro_bias_by) = 0;
	dx(X::gyro_bias_bz) = 0;
	dx(X::accel_scale) = 0;
	dx(X::pos_n) = x(X::vel_n);
	dx(X::pos_e) = x(X::vel_e);
	dx(X::pos_d) = x(X::vel_d);
	dx(X::terrain_alt) = 0;
	dx(X::baro_bias) = 0;
	dx(X::wind_n) = 0;
	dx(X::wind_e) = 0;
	dx(X::wind_d) = 0;
	return dx;
}

void IEKF::callbackImu(const sensor_combined_s *msg)
{
	//ROS_INFO("imu callback");
	_u(U::omega_nb_bx) = msg->gyro_rad[0];
	_u(U::omega_nb_by) = msg->gyro_rad[1];
	_u(U::omega_nb_bz) = msg->gyro_rad[2];
	_u(U::accel_bx) = msg->accelerometer_m_s2[0];
	_u(U::accel_by) = msg->accelerometer_m_s2[1];
	_u(U::accel_bz) = msg->accelerometer_m_s2[2];

	// predict driven by gyro callback
	if (msg->gyro_integral_dt > 0) {
		predict(msg->gyro_integral_dt);
	};

	// correct  if new data
	correctAccel(msg);

	correctMag(msg);

	correctBaro(msg);

	publish();
}

void IEKF::correctAccel(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampAccelNew = msg->timestamp + msg->accelerometer_timestamp_relative;

	if (timestampAccelNew != _timestampAccel) {
		dt = (timestampAccelNew - _timestampAccel) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampAccel = timestampAccelNew;

	} else {
		return;
	}

	// measurement
	Vector3f y_b(
		msg->accelerometer_m_s2[0],
		msg->accelerometer_m_s2[1],
		msg->accelerometer_m_s2[2]);

	// don't correct if accelerating
	float relNormError = (Vector3f(y_b / _x(X::accel_scale)).norm()
			      - _g_n.norm()) / _g_n.norm();

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_g_n = q_nb.conjugate(y_b / _x(X::accel_scale));
	Vector3f r = y_g_n - _g_n;

	// define R
	// worst accel dir change is if accel is normal to gravity,
	// assume this and calculate angle covariance based on accel norm error
	Matrix<float, Y_accel::n, Y_accel::n> R;
	R(Y_accel::accel_bx, Y_accel::accel_bx) = 1e-2f / dt + relNormError * relNormError;
	R(Y_accel::accel_by, Y_accel::accel_by) = 1e-2f / dt + relNormError * relNormError;
	R(Y_accel::accel_bz, Y_accel::accel_bz) = 1e-2f / dt + relNormError * relNormError;

	// define H
	Matrix<float, Y_accel::n, Xe::n> H;
	Matrix3f tmp = _g_n.unit().hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_accel::accel_bx + i, Xe::rot_n + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_accel::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_accel::n]) {
		ROS_DEBUG("accel fault");
		return;
	}

	//ROS_INFO("accel correction");
	//dxe.print();

	// don't allow yaw correction
	//dxe(Xe::rot_d) = 0;

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Quatf q_nb2(x(X::q_nb_0), x(X::q_nb_1),
		    x(X::q_nb_2), x(X::q_nb_3));
	Vector3f r2 = q_nb2.conjugate(y_b / x(X::accel_scale)) - _g_n;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_INFO("accel linearization error!!!!!!!!!!!!!!!!!!!!");
		Vector3f rot(dxe(Xe::rot_n), dxe(Xe::rot_e), dxe(Xe::rot_d));
		float angle = rot.norm();
		float angle_max = acosf(y_g_n.dot(_g_n)) / y_g_n.norm() / _g_n.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_g_n.cross(_g_n).unit();
		dxe.setZero();
		dxe(Xe::rot_n) = axis(0) * angle;
		dxe(Xe::rot_e) = axis(1) * angle;
		dxe(Xe::rot_d) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);

	} else {
		setX(x);
	}

	setP(_P + dP);
}

void IEKF::correctMag(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampMagNew = msg->timestamp + msg->magnetometer_timestamp_relative;

	if (timestampMagNew != _timestampMag) {
		dt = (timestampMagNew - _timestampMag) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampMag = timestampMagNew;

	} else {
		return;
	}

	// calculate residual
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1),
		   _x(X::q_nb_2), _x(X::q_nb_3));
	Vector3f y_b = Vector3f(
			       msg->magnetometer_ga[0],
			       msg->magnetometer_ga[1],
			       msg->magnetometer_ga[2]).unit();
	Vector3f yh = _B_n.unit();
	Vector3f y = q_nb.conjugate(y_b);
	Vector3f r = y - yh;

	// define R
	Matrix<float, Y_mag::n, Y_mag::n> R;
	R(Y_mag::mag_n, Y_mag::mag_n) = 2e-3f / dt;
	R(Y_mag::mag_e, Y_mag::mag_e) = 2e-3f / dt;
	R(Y_mag::mag_d, Y_mag::mag_d) = 20e-3f / dt; // less certain about local inclination

	// define H
	Matrix<float, Y_mag::n, Xe::n> H;
	Matrix3f tmp = yh.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			H(Y_mag::mag_n + i, Xe::rot_n + j) = tmp(i, j);
		}
	}

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_mag::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_mag::n]) {
		ROS_DEBUG("mag fault");
	}

	//ROS_INFO("mag correction");
	//dxe.print();

	// don't allow roll/ pitch correction
	//dxe(Xe::rot_n) = 0;
	//dxe(Xe::rot_e) = 0;

	Vector<float, X::n> x = applyErrorCorrection(dxe);
	Vector3f r2 = Quatf(x(X::q_nb_0), x(X::q_nb_1),
			    x(X::q_nb_2), x(X::q_nb_3)).conjugate(y_b) - yh;

	if (r2.norm() - r.norm() > 1e-2f) {
		ROS_INFO("mag linearization error!!!!!!!!!!!!!!!!!!!!");
		Vector3f rot(dxe(Xe::rot_n), dxe(Xe::rot_e), dxe(Xe::rot_d));
		Vector3f y_xy = Vector3f(y(0), y(1), 0);
		Vector3f yh_xy = Vector3f(yh(0), yh(1), 0);
		float angle = rot.norm();
		float angle_max = acosf(y_xy.dot(yh_xy)) / y_xy.norm() / yh_xy.norm();

		if (angle > angle_max) {
			angle = angle_max;
		}

		Vector3f axis = y_xy.cross(yh_xy).unit();
		dxe.setZero();
		dxe(Xe::rot_n) = axis(0) * angle;
		dxe(Xe::rot_e) = axis(1) * angle;
		dxe(Xe::rot_d) = axis(2) * angle;
		x = applyErrorCorrection(dxe);
		setX(x);

	} else {
		setX(x);
	}

	setP(_P + dP);
}

void IEKF::correctBaro(const sensor_combined_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampBaroNew = msg->timestamp + msg->baro_timestamp_relative;

	if (timestampBaroNew != _timestampBaro) {
		dt = (timestampBaroNew - _timestampBaro) / 1.0e6f;

		if (dt < 0) {
			return;
		}

		_timestampBaro = timestampBaroNew;

	} else {
		return;
	}

	// calculate residual
	Vector<float, Y_baro::n> y;
	y(Y_baro::asl) = msg->baro_alt_meter;
	Vector<float, Y_baro::n> yh;
	yh(Y_baro::asl)	= -_x(X::pos_d) + _x(X::baro_bias) - _origin.getAlt();
	Vector<float, Y_baro::n> r = y - yh;

	// define R
	Matrix<float, Y_baro::n, Y_baro::n> R;
	R(Y_baro::asl, Y_baro::asl) = 3e-3f / dt;

	// define H
	Matrix<float, Y_baro::n, Xe::n> H;
	H(Y_baro::asl, Xe::pos_d) = -1;
	H(Y_baro::asl, Xe::baro_bias) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_baro::n>(_P, H, R, r, dxe, dP, beta);

	//ROS_INFO("baro residual");
	//r.print();

	if (beta > BETA_TABLE[Y_baro::n]) {
		ROS_DEBUG("baro fault");
	}

	//ROS_INFO("baro correction");
	//dxe.print();

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
	//dxe(Xe::rot_n) = 0;
	//dxe(Xe::rot_e) = 0;
	//dxe(Xe::rot_d) = 0;
	//dxe(Xe::gyro_bias_n) = 0;
	//dxe(Xe::gyro_bias_e) = 0;
	//dxe(Xe::gyro_bias_d) = 0;
}

void IEKF::correctGps(const vehicle_gps_position_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampGpsNew = msg->timestamp;

	if (timestampGpsNew == _timestampGps) {
		return;
	}

	dt = (timestampGpsNew - _timestampGps) / 1.0e6f;

	if (dt < 0) {
		return;
	}

	_timestampGps = timestampGpsNew;


	// check for good gps signal
	if (msg->satellites_used < 6 || msg->fix_type < 3) {
		return;
	}

	_timestampGps = msg->timestamp;
	double lat_deg = msg->lat * 1e-7;
	double lon_deg = msg->lon * 1e-7;
	float alt_m = msg->alt * 1e-3;
	float vel_stddev = sqrt(
				   _P(Xe::vel_n, Xe::vel_n)
				   + _P(Xe::vel_e, Xe::vel_e)
				   + _P(Xe::vel_d, Xe::vel_d));

	// init global reference
	if (!_origin.xyInitialized() && vel_stddev < 1.0f) {
		ROS_INFO("gps map ref init %12.6f %12.6f", double(lat_deg), double(lon_deg));
		_origin.xyInitialize(lat_deg, lon_deg, msg->timestamp);
	}

	if (!_origin.altInitialized() && _P(Xe::pos_d, Xe::pos_d) < 1.0f) {
		ROS_INFO("gps alt init %12.2f", double(alt_m));
		_origin.altInitialize(alt_m, msg->timestamp);
	}

	// calculate residual
	float gps_pos_n = 0;
	float gps_pos_e = 0;
	float gps_pos_d = 0;
	_origin.globalToLocal(lat_deg, lon_deg, alt_m,
			      gps_pos_n, gps_pos_e, gps_pos_d);

	Vector<float, Y_gps::n> y;
	y(Y_gps::pos_n) = gps_pos_n;
	y(Y_gps::pos_e) = gps_pos_e;
	y(Y_gps::pos_d) = gps_pos_d;
	y(Y_gps::vel_n) = msg->vel_n_m_s;
	y(Y_gps::vel_e) = msg->vel_e_m_s;
	y(Y_gps::vel_d) = msg->vel_d_m_s;

	Vector<float, Y_gps::n> yh;
	yh(Y_gps::pos_n) = _x(X::pos_n);
	yh(Y_gps::pos_e) = _x(X::pos_e);
	yh(Y_gps::pos_d) = _x(X::pos_d);
	yh(Y_gps::vel_n) = _x(X::vel_n);
	yh(Y_gps::vel_e) = _x(X::vel_e);
	yh(Y_gps::vel_d) = _x(X::vel_d);

	Vector<float, Y_gps::n> r = y - yh;

	// define R
	Matrix<float, Y_gps::n, Y_gps::n> R;
	R(Y_gps::pos_n, Y_gps::pos_n) = 0.2f / dt;
	R(Y_gps::pos_e, Y_gps::pos_e) = 0.2f / dt;
	R(Y_gps::pos_d, Y_gps::pos_d) = 0.2f / dt;
	R(Y_gps::vel_n, Y_gps::vel_n) = 0.2f / dt;
	R(Y_gps::vel_e, Y_gps::vel_e) = 0.2f / dt;
	R(Y_gps::vel_d, Y_gps::vel_d) = 0.2f / dt;

	// define H
	Matrix<float, Y_gps::n, Xe::n> H;
	H(Y_gps::pos_n, Xe::pos_n) = 1;
	H(Y_gps::pos_e, Xe::pos_e) = 1;
	H(Y_gps::pos_d, Xe::pos_d) = 1;
	H(Y_gps::vel_n, Xe::vel_n) = 1;
	H(Y_gps::vel_e, Xe::vel_e) = 1;
	H(Y_gps::vel_d, Xe::vel_d) = 1;

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_gps::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_gps::n]) {
		ROS_DEBUG("gps fault");
	}

	//ROS_INFO("gps correction");
	//dxe.print();

	//ROS_INFO("gps rot correct %10.4f %10.4f %10.4f",
	//double(dxe(Xe::rot_n)),
	//double(dxe(Xe::rot_e)),
	//double(dxe(Xe::rot_d)));
	//dxe(Xe::rot_n) = 0;
	//dxe(Xe::rot_e) = 0;
	//dxe(Xe::rot_d) = 0;
	//dxe(Xe::gyro_bias_n) = 0;
	//dxe(Xe::gyro_bias_e) = 0;
	//dxe(Xe::gyro_bias_d) = 0;
	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}


void IEKF::correctAirspeed(const airspeed_s *msg)
{
	// return if no new data
	float dt = 0;
	uint64_t timestampAirspeedNew = msg->timestamp;

	if (timestampAirspeedNew == _timestampAirspeed) {
		return;
	}

	dt = (timestampAirspeedNew - _timestampAirspeed) / 1.0e6f;

	if (dt < 0) {
		return;
	}

	_timestampAirspeed = timestampAirspeedNew;

	// attitude info
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Dcmf C_nb = q_nb;

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_n), _x(X::wind_e), _x(X::wind_d));
	Vector3f vel_n(_x(X::vel_n), _x(X::vel_d), _x(X::vel_d));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float yh = wind_rel_b(0); // body x component aligned with pitot tube

	// measured airspeed
	float y = msg->true_airspeed_unfiltered_m_s;

	Vector<float, 1> r;
	r(0) = y - yh;

	// define R
	Matrix<float, Y_airspeed::n, Y_airspeed::n> R;
	R(Y_airspeed::airspeed, Y_airspeed::airspeed) = 1.0f / dt;

	// define H
	// TODO make this invariant
	Matrix<float, Y_airspeed::n, Xe::n> H;
	H(Y_airspeed::airspeed, Xe::vel_n) = C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::vel_e) = C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::vel_d) = C_nb(2, 0);
	H(Y_airspeed::airspeed, Xe::wind_n) = -C_nb(0, 0);
	H(Y_airspeed::airspeed, Xe::wind_e) = -C_nb(1, 0);
	H(Y_airspeed::airspeed, Xe::wind_d) = -C_nb(2, 0);

	// kalman correction
	Vector<float, Xe::n> dxe;
	SquareMatrix<float, Xe::n> dP;
	float beta = 0;
	kalman_correct<float, Xe::n, Y_airspeed::n>(_P, H, R, r, dxe, dP, beta);

	if (beta > BETA_TABLE[Y_airspeed::n]) {
		ROS_DEBUG("airspeed fault");
	}

	//ROS_INFO("airspeed correction");
	//dxe.print();

	setX(applyErrorCorrection(dxe));
	setP(_P + dP);
}

void IEKF::predict(float dt)
{
	// define process noise matrix
	Matrix<float, Xe::n, Xe::n> Q;
	Q(Xe::rot_n, Xe::rot_n) = 1e-6;
	Q(Xe::rot_e, Xe::rot_e) = 1e-6;
	Q(Xe::rot_d, Xe::rot_d) = 1e-6;
	Q(Xe::vel_n, Xe::vel_n) = 1e-3;
	Q(Xe::vel_e, Xe::vel_e) = 1e-3;
	Q(Xe::vel_d, Xe::vel_d) = 1e-3;
	Q(Xe::gyro_bias_n, Xe::gyro_bias_n) = 1e-6;
	Q(Xe::gyro_bias_e, Xe::gyro_bias_e) = 1e-6;
	Q(Xe::gyro_bias_d, Xe::gyro_bias_d) = 1e-6;
	Q(Xe::accel_scale, Xe::accel_scale) = 1e-6;
	Q(Xe::pos_n, Xe::pos_n) = 1e-6;
	Q(Xe::pos_e, Xe::pos_e) = 1e-6;
	Q(Xe::pos_d, Xe::pos_d) = 1e-6;
	Q(Xe::terrain_alt, Xe::terrain_alt) = 1e-1f;
	Q(Xe::baro_bias, Xe::baro_bias) = 1e-3f;
	Q(Xe::wind_n, Xe::wind_n) = 1e-3f;
	Q(Xe::wind_e, Xe::wind_e) = 1e-3f;
	Q(Xe::wind_d, Xe::wind_d) = 1e-3f;

	// define A matrix
	Matrix<float, Xe::n, Xe::n> A;

	// derivative of rotation error is -0.5 * gyro bias
	A(Xe::rot_n, Xe::Xe::gyro_bias_n) = -0.5;
	A(Xe::rot_e, Xe::Xe::gyro_bias_e) = -0.5;
	A(Xe::rot_d, Xe::Xe::gyro_bias_d) = -0.5;

	// derivative of velocity
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));

	if (fabsf(q_nb.norm() - 1.0f) > 1e-3f) {
		ROS_DEBUG("normalizing quaternion, norm was %6.4f\n", double(q_nb.norm()));
		q_nb.normalize();
		_x(X::q_nb_0) = q_nb(0);
		_x(X::q_nb_1) = q_nb(1);
		_x(X::q_nb_2) = q_nb(2);
		_x(X::q_nb_3) = q_nb(3);
	}

	Vector3f a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3f J_a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	Matrix<float, 3, 3> a_tmp = -J_a_n.hat() * 2;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::vel_n + i, Xe::rot_n + j) = a_tmp(i, j);
		}

		A(Xe::vel_n + i, Xe::accel_scale) = -J_a_n(i);
	}

	// derivative of gyro bias
	Vector3f omega_nb_b(
		_u(U::omega_nb_bx), _u(U::omega_nb_by), _u(U::omega_nb_bz));
	Vector3f gyro_bias_b(
		_x(X::gyro_bias_bx), _x(X::gyro_bias_by), _x(X::gyro_bias_bz));
	Vector3f J_omega_n = q_nb.conjugate(omega_nb_b - gyro_bias_b);
	Matrix<float, 3, 3> g_tmp = J_omega_n.hat();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			A(Xe::gyro_bias_n + i, Xe::rot_n + j) = g_tmp(i, j);
		}
	}

	// derivative of position is velocity
	A(Xe::pos_n, Xe::vel_n) = 1;
	A(Xe::pos_e, Xe::vel_e) = 1;
	A(Xe::pos_d, Xe::vel_d) = 1;

	// derivative of terrain alt is zero

	// derivative of baro bias is zero

	//ROS_INFO("A:");
	//for (int i=0;i<Xe::n; i++) {
	//for (int j=0;j<Xe::n; j++) {
	//printf("%10.3f, ", double(A(i, j)));
	//}
	//printf("\n");
	//}

	// continuous time kalman filter prediction
	// integrate runge kutta 4th order
	// TODO move rk4 algorithm to matrixlib
	// https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
	float h = dt;
	Vector<float, X::n> k1, k2, k3, k4;
	k1 = dynamics(0, _x, _u);
	k2 = dynamics(h / 2, _x + k1 * h / 2, _u);
	k3 = dynamics(h / 2, _x + k2 * h / 2, _u);
	k4 = dynamics(h, _x + k3 * h, _u);
	Vector<float, X::n> dx = (k1 + k2 * 2 + k3 * 2 + k4) * (h / 6);

	//ROS_INFO("dx predict \n");
	//dx.print();
	setX(_x + dx);

	// propgate covariance using euler integration
	Matrix<float, Xe::n, Xe::n> dP = (A * _P + _P * A.T() + Q) * dt;
	setP(_P + dP);

	//ROS_INFO("P:");
	//_P.print();

}

Vector<float, X::n> IEKF::applyErrorCorrection(const Vector<float, Xe::n> &d_xe)
{
	Quatf q_nb(_x(X::q_nb_0), _x(X::q_nb_1), _x(X::q_nb_2), _x(X::q_nb_3));
	Quatf d_q_nb = Quatf(0,
			     d_xe(Xe::rot_n), d_xe(Xe::rot_e), d_xe(Xe::rot_d)) * q_nb;
	//ROS_INFO("d_q_nb");
	//d_q_nb.print();
	Vector3f d_gyro_bias_b = q_nb.conjugate_inversed(
					 Vector3f(d_xe(Xe::gyro_bias_n),
							 d_xe(Xe::gyro_bias_e),
							 d_xe(Xe::gyro_bias_d)));

	// linear term correction is the same
	// as the error correction
	Vector<float, X::n> x = _x;
	x(X::q_nb_0) += d_q_nb(0);
	x(X::q_nb_1) += d_q_nb(1);
	x(X::q_nb_2) += d_q_nb(2);
	x(X::q_nb_3) += d_q_nb(3);
	x(X::vel_n) += d_xe(Xe::vel_n);
	x(X::vel_e) += d_xe(Xe::vel_e);
	x(X::vel_d) += d_xe(Xe::vel_d);
	x(X::gyro_bias_bx) += d_gyro_bias_b(0);
	x(X::gyro_bias_by) += d_gyro_bias_b(1);
	x(X::gyro_bias_bz) +=  d_gyro_bias_b(2);
	x(X::accel_scale) += _x(X::accel_scale) * d_xe(Xe::accel_scale);
	x(X::pos_n) += d_xe(Xe::pos_n);
	x(X::pos_e) += d_xe(Xe::pos_e);
	x(X::pos_d) += d_xe(Xe::pos_d);
	x(X::terrain_alt) += d_xe(Xe::terrain_alt);
	x(X::baro_bias) += d_xe(Xe::baro_bias);
	x(X::wind_n) += d_xe(Xe::wind_n);
	x(X::wind_e) += d_xe(Xe::wind_e);
	x(X::wind_d) += d_xe(Xe::wind_d);
	return x;
}

void IEKF::setP(const SquareMatrix<float, Xe::n> &P)
{
	_P = P;

	for (int i = 0; i < Xe::n; i++) {
		// only operate on upper triangle, then copy to lower

		// don't allow NaN or large numbers
		for (int j = 0; j <= i; j++) {
			if (!PX4_ISFINITE(_P(i, j))) {
				ROS_DEBUG("P(%d, %d) NaN, resetting", i, j);

				if (i == j) {
					_P(i, j) = _P0Diag(i);

				} else {
					_P(i, j) = 0;
				}
			}

			if (_P(i, j) > 1e6f) {
				// upper bound
				_P(i, j) = 1e6f;
			}
		}

		// force positive diagonal
		if (_P(i, i) < 1e-6f) {
			ROS_DEBUG("P(%d, %d) < 1e-6, setting to 1e-6", i, i);
			_P(i, i) = 1e-6f;
		}

		// force symmetry, copy uppper triangle to lower
		for (int j = 0; j < i; j++) {
			_P(j, i) = _P(i, j);
		}
	}
}

void IEKF::setX(const Vector<float, X::n> &x)
{
	// set private state
	_x = x;

	// for quaterinons we bound at 2
	// so that saturation doesn't change
	// the direction of the vectors typicall
	// and normalization
	// handles small errors
	Vector<float, X::n> lowerBound;
	lowerBound(X::q_nb_0) = -2;
	lowerBound(X::q_nb_1) = -2;
	lowerBound(X::q_nb_2) = -2;
	lowerBound(X::q_nb_3) = -2;
	lowerBound(X::vel_n) = -100;
	lowerBound(X::vel_e) = -100;
	lowerBound(X::vel_d) = -100;
	lowerBound(X::gyro_bias_bx) = 0;
	lowerBound(X::gyro_bias_by) = 0;
	lowerBound(X::gyro_bias_bz) = 0;
	lowerBound(X::accel_scale) = 0.8;
	lowerBound(X::pos_n) = -1e9;
	lowerBound(X::pos_e) = -1e9;
	lowerBound(X::pos_d) = -1e9;
	lowerBound(X::terrain_alt) = -1e6;
	lowerBound(X::baro_bias) = -1e6;

	Vector<float, X::n> upperBound;
	upperBound(X::q_nb_0) = 2;
	upperBound(X::q_nb_1) = 2;
	upperBound(X::q_nb_2) = 2;
	upperBound(X::q_nb_3) = 2;
	upperBound(X::vel_n) = 100;
	upperBound(X::vel_e) = 100;
	upperBound(X::vel_d) = 100;
	upperBound(X::gyro_bias_bx) = 0;
	upperBound(X::gyro_bias_by) = 0;
	upperBound(X::gyro_bias_bz) = 0;
	upperBound(X::accel_scale) = 1.5;
	upperBound(X::pos_n) = 1e9;
	upperBound(X::pos_e) = 1e9;
	upperBound(X::pos_d) = 1e9;
	upperBound(X::terrain_alt) = 1e6;
	upperBound(X::baro_bias) = 1e6;

	for (int i = 0; i < X::n; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			ROS_WARN("x(%d) NaN, setting to %10.4f", i, double(_x0(i)));
			_x(i) = _x0(i);
		}

		if (_x(i) < lowerBound(i)) {
			//ROS_INFO("x(%d) < lower bound, saturating", i);
			_x(i) = lowerBound(i);

		} else if (_x(i) > upperBound(i)) {
			//ROS_INFO("x(%d) > upper bound, saturating", i);
			_x(i) = upperBound(i);
		}
	}
}

void IEKF::publish()
{
	//ROS_INFO("x:");
	//_x.print();

	//ROS_INFO("P:");
	//_P.diag().print();

	float eph = sqrt(_P(Xe::pos_n, Xe::pos_n) + _P(Xe::pos_e, Xe::pos_e));
	float epv = _P(Xe::pos_d, Xe::pos_d);
	Quatf q_nb(
		_x(X::q_nb_0), _x(X::q_nb_1),
		_x(X::q_nb_2), _x(X::q_nb_3));
	Euler<float> euler_nb = q_nb;
	Vector3f a_b(_u(U::accel_bx), _u(U::accel_by), _u(U::accel_bz));
	Vector3f a_n = q_nb.conjugate(a_b / _x(X::accel_scale));
	ros::Time now = ros::Time::now();

	// predicted airspeed
	Vector3f wind_n(_x(X::wind_n), _x(X::wind_e), _x(X::wind_d));
	Vector3f vel_n(_x(X::vel_n), _x(X::vel_d), _x(X::vel_d));
	Vector3f wind_rel_b = q_nb.conjugate_inversed(wind_n - vel_n);
	float airspeed = wind_rel_b(0); // body x component aligned with pitot tube

	float vel_stddev = sqrt(
				   _P(Xe::vel_n, Xe::vel_n)
				   + _P(Xe::vel_e, Xe::vel_e)
				   + _P(Xe::vel_d, Xe::vel_d));

	// publish attitude
	{
		vehicle_attitude_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.rollspeed = _u(U::omega_nb_bx) - _x(X::gyro_bias_bx);
		msg.pitchspeed = _u(U::omega_nb_by) - _x(X::gyro_bias_by);
		msg.yawspeed = _u(U::omega_nb_bz) - _x(X::gyro_bias_bz);
		_pubAttitude.publish(msg);
	}

	// publish local position
	if (vel_stddev < 1.0f) {
		vehicle_local_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.xy_valid = true;
		msg.z_valid = true;
		msg.v_xy_valid = true;
		msg.v_z_valid = true;
		msg.x = _x(X::pos_n);
		msg.y = _x(X::pos_e);
		msg.z = _x(X::pos_d);
		msg.delta_xy[0] = 0;
		msg.delta_xy[1] = 0;
		msg.delta_z = 0;
		msg.vx = _x(X::vel_n);
		msg.vy = _x(X::vel_e);
		msg.vz = _x(X::vel_d);
		msg.delta_vxy[0] = 0;
		msg.delta_vxy[1] = 0;
		msg.delta_vz = 0;
		msg.xy_reset_counter = 0;
		msg.z_reset_counter = 0;
		msg.vxy_reset_counter = 0;
		msg.vz_reset_counter = 0;
		msg.yaw = euler_nb(2);
		msg.xy_global = _origin.xyInitialized();
		msg.z_global = _origin.altInitialized();
		msg.ref_timestamp = _origin.getXYTimestamp();
		msg.ref_lat = _origin.getLatDeg();
		msg.ref_lon = _origin.getLonDeg();
		msg.ref_alt = _origin.getAlt();
		msg.dist_bottom = -_x(X::pos_d) - _x(X::terrain_alt);
		msg.dist_bottom_rate = -_x(X::vel_d);
		msg.surface_bottom_timestamp = 0;
		msg.dist_bottom_valid = true;
		msg.eph = eph;
		msg.epv = epv;
		_pubLocalPosition.publish(msg);
	}

	// publish global position
	if (vel_stddev < 1.0f && _origin.xyInitialized() & _origin.altInitialized()) {
		double lat_deg = 0;
		double lon_deg = 0;
		float alt_m = 0;
		_origin.localToGlobal(_x(X::pos_n), _x(X::pos_e), _x(X::pos_d), lat_deg, lon_deg, alt_m);
		vehicle_global_position_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.time_utc_usec = 0; // TODO
		msg.lat = lat_deg;
		msg.lon = lon_deg;
		msg.alt = alt_m;
		msg.delta_lat_lon[0] = 0;
		msg.delta_lat_lon[1] = 0;
		msg.delta_alt = 0;
		msg.lat_lon_reset_counter = 0;
		msg.alt_reset_counter = 0;
		msg.vel_n = _x(X::vel_n);
		msg.vel_e = _x(X::vel_e);
		msg.vel_d = _x(X::vel_d);
		msg.yaw = euler_nb(2);
		msg.eph = eph;
		msg.epv = epv;
		msg.terrain_alt = _x(X::terrain_alt) + _origin.getAlt();
		msg.terrain_alt_valid = true;
		msg.dead_reckoning = false;
		msg.pressure_alt = alt_m; // TODO
		_pubGlobalPosition.publish(msg);
	}

	// publish control state
	{
		// specific acceleration
		control_state_s msg = {};
		msg.timestamp = now.toNSec() / 1e3;
		msg.x_acc = a_n(0);
		msg.y_acc = a_n(1);
		msg.z_acc = a_n(2);
		msg.x_vel = _x(X::vel_n);
		msg.y_vel = _x(X::vel_e);
		msg.z_vel = _x(X::vel_d);
		msg.x_pos = _x(X::pos_n);
		msg.y_pos = _x(X::pos_e);
		msg.z_pos = _x(X::pos_d);
		msg.airspeed = airspeed;
		msg.airspeed_valid = vel_stddev < 1.0f;
		msg.vel_variance[0] = _P(Xe::vel_n, Xe::vel_n);
		msg.vel_variance[1] = _P(Xe::vel_e, Xe::vel_e);
		msg.vel_variance[2] = _P(Xe::vel_d, Xe::vel_d);
		msg.pos_variance[0] = _P(Xe::pos_n, Xe::pos_n);
		msg.pos_variance[1] = _P(Xe::pos_e, Xe::pos_e);
		msg.pos_variance[2] = _P(Xe::pos_d, Xe::pos_d);
		msg.q[0] = _x(X::q_nb_0);
		msg.q[1] = _x(X::q_nb_1);
		msg.q[2] = _x(X::q_nb_2);
		msg.q[3] = _x(X::q_nb_3);
		msg.delta_q_reset[0] = 0;
		msg.delta_q_reset[1] = 0;
		msg.delta_q_reset[2] = 0;
		msg.delta_q_reset[3] = 0;
		msg.quat_reset_counter = 0;
		msg.roll_rate = _u(U::omega_nb_bx) - _x(X::gyro_bias_bx);
		msg.pitch_rate = _u(U::omega_nb_by) - _x(X::gyro_bias_by);
		msg.yaw_rate = _u(U::omega_nb_bz) - _x(X::gyro_bias_bz);
		msg.horz_acc_mag = 0;
		_pubControlState.publish(msg);
	}

	// estimator status
	{
		estimator_status_s msg = {};
		msg.timestamp = now.toNSec();
		msg.vibe[0] = 0; // TODO
		msg.vibe[1] = 0; // TODO
		msg.vibe[2] = 0; // TODO
		msg.n_states = X::n;

		for (int i = 0; i < X::n; i++) {
			msg.states[i] = _x(i);
			// offset by 1 so covariances lined up with states
			// except for quaternoin and rotations error
		}

		for (int i = 0; i < Xe::n; i++) {
			msg.covariances[i] = _P(i, i);
		}

		msg.gps_check_fail_flags = 0; // TODO
		msg.control_mode_flags = 0; // TODO
		msg.filter_fault_flags = 0; // TODO
		msg.pos_horiz_accuracy = eph;
		msg.pos_vert_accuracy = epv;
		msg.innovation_check_flags = 0; // TODO
		msg.mag_test_ratio = 0; // TODO
		msg.vel_test_ratio = 0; // TODO
		msg.pos_test_ratio = 0; // TODO
		msg.hgt_test_ratio = 0; // TODO
		msg.tas_test_ratio = 0; // TODO
		msg.hagl_test_ratio = 0; // TODO
		msg.solution_status_flags = 0; // TODO
		_pubEstimatorStatus.publish(msg);
	}
}
