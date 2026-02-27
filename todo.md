# Project Summary — Channel Modeling with Ray Tracing (ELEC-H415)

## Objective

Develop a **2D ray-tracing simulator** to model electromagnetic wave propagation at **26 GHz** for a **5G small cell base station (BS)**.

The simulator must:

* Compute the **received power** at a user equipment (UE),
* Determine the **coverage area**,
* Estimate the **achievable bit rate** as a function of UE location.

---

## Main Assumptions

* **Far-field approximation** (locally plane waves). **=> remove near-field from results?!**
* 2D horizontal propagation only (BS and UE at same height, same plane).
* No diffraction (due to high frequency).
* Materials: _all_ have relative permittivity of $\varepsilon_r = 4$.
* BS:

  * 26 GHz
  * Bandwidth = 200 MHz
  * Tx power = 20 dBm
  * $\lambda$/2 vertical dipoles (lossless)

Considered propagation mechanisms:

* Direct path
* Up to **three reflections**
* Wall transmission (but not through entire buildings)

---

## Coverage & Data Rate Model

Communication is possible only if:

$P_{RX} > \text{UE sensitivity}$

Sensitivity depends **linearly (in log scale)** on bit rate:

| Sensitivity | Bit rate |
| ----------- | -------- |
| −80 dBm     | 100 Mb/s |
| −60 dBm     | 1 Gb/s   |

* Below −80 dBm → no communication
* Above −60 dBm → data rate capped

---

## Environment

* Freely chosen simplified outdoor geometry.
* Walls have no thickness in geometry (only in EM calculations for coefficients).
* Environment is discretized into ~1sqm local areas.

---

## Required Results

1. **Validate the simulator**

   * Free-space case
   * Single reflection case
   * **Then** up to three reflections

2. Use the **image method** to find propagation paths.

3. For each local area:

   * Compute average received power $⟨P_{RX}⟩$
   * Plot:

     * Power map
     * Data rate map
   * Deduce coverage area

4. Extract a **path loss model**

   * Path loss law
   * Standard deviation around it
   * Discuss statistical behavior

5. (Optional bonus)

   * Multi-BS deployment optimization (UE connects to BS with most power, do not add up power from two BSs)
   * Recursive reflections (for 4+ reflections)
   * Floor reflection
   * More advanced channel models (TDL, impulse responses, etc.)
   * 3D (BS and UE not on same plane)

---

## Final Goal

Produce a **surrogate(simplified) channel model**. Adding extra features is a welcome bonus.

* Deliverable: **PDF report, and documented code** on UV
