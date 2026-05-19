#ifndef PARAMETERS_H
#define PARAMETERS_H

#include <complex>
#include <QtTypes>
#include <qmath.h>

using namespace std;

static constexpr complex<qreal> j(0, 1); // useful
//static constexpr qulonglong c = 3*1e8;
static constexpr qulonglong c = 299792458; // is more precise needed ?

static constexpr qreal epsilon_0 = 8.854187817e-12;
static constexpr qreal mu_0 = 1.256637e-6;//4 * M_PI * 1e-7; // 1.256637e-6; // ?

// 5G
static constexpr qreal freq = 26e9; // 26 GHz (f)
static constexpr qreal f = freq;
constexpr qreal bandwidth = 200e6; // 200MHz (B) // TODO: impl
constexpr qreal B = bandwidth;
static constexpr qreal omega = 2 * M_PI * freq; // pulse
static constexpr qreal wavelength = c/freq;
static constexpr qreal lambda = wavelength;
static const qreal Z_0 = sqrt(mu_0 / epsilon_0); // vacuum impedance

static constexpr qreal epsilon_r = 4; // relative permittivity of ALL materials
static constexpr qreal sigma_cond = 0; // lossless material (to use dielectric slab model for walls)

/*
 Sensitivity | Bit rate
     -80 dBm | 100 Mb/s (lower: comms impossible)
     -60 dBm | 1 Gb/s  (higher: data rate capped)
*/
static constexpr qreal min_sensitivity_dBm = -80; // minimum sensitivity in dBm (communication impossible)
static constexpr qreal max_sensitivity_dBm = -60; // maximum sensitivity in dBm (max capped data rate)
static constexpr qreal min_bitrate_bps = 100e6; // 100 Mbps
static constexpr qreal max_bitrate_bps = 1e9; // 1 Gbps
static constexpr qreal max_bitrate_Mbps = 1000; // 1 Gbps
static constexpr qreal min_bitrate_Mbps = 100; // 100 Mbps
// assume that the UE sensitivity (in dBm) varies linearly with the maximum achievable bit rate when both quantities are expressed in a logarithmic scale

// far-field, plane waves, check:
static constexpr qreal D = 0.1; // max physical dimension of antenna e.g. ~= 10cm
static const qreal far_field_min_distance = qMax(qMax(1.6*lambda,5*D),2*qPow(D,2)/lambda); // r_{ff} border of exclusion zone

// TODO: change
static constexpr qreal max_x = 120;
static constexpr qreal min_x = 0;
static constexpr qreal min_y = 0;
static constexpr qreal max_y = 70;
// resolution is set in simulation->resolution, user chosen at runtime

static constexpr qreal beta_0 = 2*M_PI*freq/c; // beta

// TX and RX antennas are lossless lambda/2 dipoles
static constexpr qreal G_TX = 1.64; // transmitter antenna gain, 1.64 or 1.7
static constexpr qreal G_RX = 1.64; // same as TX
constexpr qreal P_TX = 0.1; // transmitter power in Watts
constexpr qreal P_TX_dBm = 20; // transmitter power in dBm
static const qreal effective_height = lambda/M_PI; // h_{e\perp} (because halfwave dipole in horizontal plane)
static constexpr qreal R_a = 73; // antenna resistance : 73 Ohm

static constexpr qreal initial_BS_x = 40;
static constexpr qreal initial_BS_y = 25;

#endif //PARAMETERS_H
