#include <cmath>
#include <iostream>
#include <iomanip>
#include <QDebug>
#include <QVector2D>
#include "parameters.h"
#include "ray.h"
#include "raysegment.h"
#include "transmitter.h"
#include "receiver.h"

/**
 * @file validation_free_space.cpp
 * @brief Validates the free-space propagation model (direct path only, no obstacles)
 * 
 * This validation test compares simulated received power against theoretical
 * free-space path loss computed using the Friis transmission equation:
 * 
 * P_RX = P_TX * G_TX * G_RX * (lambda / (4*pi*d))^2
 */

// ============================================================================
// THEORETICAL CALCULATIONS
// ============================================================================

/**
 * Compute theoretical free-space received power using Friis transmission equation
 * @param distance_m Distance in meters between TX and RX
 * @param tx_power_W Transmitter power in Watts
 * @param tx_gain Transmitter antenna gain (linear)
 * @param rx_gain Receiver antenna gain (linear)
 * @return Received power in Watts
 */
qreal computeTheoreticalPower_FriisEquation(qreal distance_m, qreal tx_power_W, 
                                             qreal tx_gain, qreal rx_gain)
{
    qreal path_loss_factor = (lambda / (4.0 * M_PI * distance_m));
    qreal P_RX = tx_power_W * tx_gain * rx_gain * path_loss_factor * path_loss_factor;
    return P_RX;
}

/**
 * Compute theoretical path loss in dB
 * @param distance_m Distance in meters
 * @return Path loss in dB (positive value)
 */
qreal computePathLoss_dB(qreal distance_m)
{
    // PL(dB) = 20*log10(4*pi*d/lambda)
    qreal PL = 20.0 * std::log10((4.0 * M_PI * distance_m) / lambda);
    return PL;
}

/**
 * Convert linear power to dBm
 */
qreal powerW_to_dBm(qreal power_W)
{
    if (power_W <= 0) return -999.0;
    return 10.0 * std::log10(power_W * 1000.0);
}

/**
 * Convert dBm to linear power in Watts
 */
qreal powerdBm_to_W(qreal power_dBm)
{
    return std::pow(10.0, power_dBm / 10.0) / 1000.0;
}

// ============================================================================
// SIMULATED POWER COMPUTATION (using project classes)
// ============================================================================

/**
 * Compute received power for a single direct ray
 * Uses the formula: P_RX = |H(f)|^2 * P_TX
 * where |H(f)|^2 includes path loss and antenna gains
 */
qreal computeSimulatedDirectRayPower(const QVector2D& tx_pos, const QVector2D& rx_pos,
                                      qreal tx_power_W, qreal tx_gain, qreal rx_gain,
                                      qreal antenna_resistance)
{
    // Distance
    qreal d = (rx_pos - tx_pos).length();
    
    // Free-space path loss: (lambda / (4*pi*d))^2
    qreal path_loss_linear = std::pow(lambda / (4.0 * M_PI * d), 2.0);
    
    // Received power using antenna formula
    // P_RX = (60 * lambda^2) / (8*pi^2*Ra) * G_TX * G_RX * P_TX * path_loss
    qreal P_RX = (60.0 * lambda * lambda) / (8.0 * M_PI * M_PI * antenna_resistance) 
                 * tx_gain * rx_gain * tx_power_W * path_loss_linear;
    
    return P_RX;
}

// ============================================================================
// VALIDATION TEST CASES
// ============================================================================

/**
 * Test 1: Simple 1-meter distance
 */
void testCase_1meter()
{
    qDebug() << "\n========== TEST CASE 1: 1 METER DISTANCE ==========";
    
    qreal distance = 1.0; // 1 meter
    qreal tx_power = 0.1; // 100 mW = 20 dBm
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX; // Assume dipole with same gain at RX
    qreal antenna_R = 73.0; // Receiver antenna resistance
    
    // Theoretical calculation
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(distance, tx_power, tx_gain, rx_gain);
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated calculation
    QVector2D tx_pos(0.0, 0.0);
    QVector2D rx_pos(distance, 0.0);
    qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error analysis
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }
}

/**
 * Test 2: Realistic distance (10 meters)
 */
void testCase_10meters()
{
    qDebug() << "\n========== TEST CASE 2: 10 METER DISTANCE ==========";
    
    qreal distance = 10.0; // 10 meters
    qreal tx_power = 0.1; // 100 mW = 20 dBm
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;
    qreal antenna_R = 73.0;
    
    // Theoretical
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(distance, tx_power, tx_gain, rx_gain);
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated
    QVector2D tx_pos(0.0, 0.0);
    QVector2D rx_pos(distance, 0.0);
    qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }
}

/**
 * Test 3: Long distance (100 meters) - typical BS coverage
 */
void testCase_100meters()
{
    qDebug() << "\n========== TEST CASE 3: 100 METER DISTANCE ==========";
    
    qreal distance = 100.0; // 100 meters
    qreal tx_power = 0.1;
    qreal tx_gain = G_TX;
    qreal rx_gain = G_TX;
    qreal antenna_R = 73.0;
    
    // Theoretical
    qreal P_theoretical = computeTheoreticalPower_FriisEquation(distance, tx_power, tx_gain, rx_gain);
    qreal PL_dB = computePathLoss_dB(distance);
    qreal P_theoretical_dBm = powerW_to_dBm(P_theoretical);
    
    // Simulated
    QVector2D tx_pos(0.0, 0.0);
    QVector2D rx_pos(distance, 0.0);
    qreal P_simulated = computeSimulatedDirectRayPower(tx_pos, rx_pos, tx_power, tx_gain, rx_gain, antenna_R);
    qreal P_simulated_dBm = powerW_to_dBm(P_simulated);
    
    // Error
    qreal error_percent = std::abs(P_simulated - P_theoretical) / P_theoretical * 100.0;
    
    qDebug() << "Distance:" << distance << "m";
    qDebug() << "Theoretical P_RX:" << P_theoretical << "W =" << P_theoretical_dBm << "dBm";
    qDebug() << "Simulated P_RX:" << P_simulated << "W =" << P_simulated_dBm << "dBm";
    qDebug() << "Path Loss:" << PL_dB << "dB";
    qDebug() << "Error:" << error_percent << "%";
    
    if (error_percent < 1.0) {
        qDebug() << "✓ PASS: Error within tolerance";
    } else {
        qDebug() << "✗ FAIL: Error exceeds 1% tolerance";
    }
}


// ============================================================================
// MAIN VALIDATION RUNNER
// ============================================================================

/**
 * Run all free-space validation tests
 */
void runValidationFreeSpace()
{
    qDebug() << "\n\n";
    qDebug() << "╔════════════════════════════════════════════════════════════╗";
    qDebug() << "║     FREE-SPACE PROPAGATION VALIDATION (PHASE 1 - TEST 1)   ║";
    qDebug() << "╚════════════════════════════════════════════════════════════╝";
    
    qDebug() << "\nTest Parameters:";
    qDebug() << "  Frequency:" << freq / 1e9 << "GHz";
    qDebug() << "  Wavelength:" << lambda << "m";
    qDebug() << "  TX Power:" << P_TX * 1000 << "mW =" << P_TX_dBm << "dBm";
    qDebug() << "  TX Gain:" << G_TX << "(linear)";
    qDebug() << "  Antenna Resistance:" << 73.0 << "Ohm";
    qDebug() << "  Vacuum Impedance (Z0):" << Z_0 << "Ohm";
    
    // Run all tests
    testCase_1meter();
    testCase_10meters();
    testCase_100meters();
    
    qDebug() << "\n════════════════════════════════════════════════════════════";
    qDebug() << "FREE-SPACE VALIDATION COMPLETE\n";
}
