Probana is a research analog/digital mixed-signal probabilistic p-bit computer. It can be used as compute backend for [p-kit](https://github.com/IBM/p-kit). Probana is designed as a specialized hardware platform for probabilistic computing, including Ising models, QUBO problems, combinatorial optimization, and other [p-kit](https://github.com/IBM/p-kit) probabilistic circuits. It also supports on-board optimization methods such as annealing, with the optimization schedule executed locally on the hardware. Probana also has some characteristics of a general-purpose analog computing platform.


## Objectives:

* develop a probabilistic computer, initially with 8 or 16 physical p-bits
* run [p-kit](https://github.com/IBM/p-kit) probabilistic circuits
* provide a full implementation using inexpensive, off-the-shelf components
* USB pluggable to any computer
* provide firmware and PCB designs
* potentially be modular, so that several 8-p-bit boards can be joined together to easily form an 8 × N probabilistic computer
* Experiment with [p-kit language models](https://github.com/IBM/p-kit/wiki/p%E2%80%90kit-Language-Model-Project) on Probana
  
## How it works or Why it should work?

The trick is to perform a calibration in the beginning. It is a software controlled correction applied in hardware. We measure the analog representation of each p-bit and we apply a software adjustment. Each p-bit implements a Bernoulli distribution. This distribution has a parameter, `p`, which defines the probability of the p-bit being `1`.

We want to preserve the unique analog randomness of each p-bit, but we also want this p-bit to follow a Bernoulli distribution with the parameter we set for it. This is achieved by measuring the relation:

$$
VBIAS \rightarrow P(1)
$$

for every physical p-bit during calibration. We call VBIAS the analog control voltage applied to a p-bit. A MCU (microcontroller such as Arduino) stores this calibration curve and later uses it in the opposite direction:

$$
\text{requested } P(1) \rightarrow VBIAS
$$

### Example 

If the software requests `P(1) = 0.7`, the MCU uses the calibration curve of that particular p-bit to adjust its `VBIAS` so that its measured output probability is approximately 0.7. In this way, it follows the required Bernoulli distribution and produces approximately 70% ones. Moreover, the physical noise remains natural and different for every p-bit, while software calibration compensates for device differences and makes each p-bit follow the probability requested by the p-kit circuit.

## Architecture

We use an MCU and a 12-bit DAC to control the probability of each physical p-bit by applying an analog control voltage that biases its random output toward 0 or 1. Each p-bit contains its own physical noise source, amplifier and comparator, producing a stochastic digital output. The MCU:

* sets the control voltage of each p-bit through the DAC
* reads the p-bit outputs
* performs the initial calibration
* applies the calibration correction so each p-bit follows the requested probability
* computes the p-kit coupling updates
* communicates with the PC over USB

The p-kit circuit (J, h) is uploaded once, then the update loop runs locally on the board. Otherwise, there would be too much communication overhead if every p-bit update had to be sent over USB individually.

```
+--------------------------------------------------------------------------------------------------------------+
|                                           MAIN PROBANA ARCHTECTURE (p-bits are not detailed)                 |
|                                                                                                              |
|  PC / p-kit                                                                                                  |
|     |                                                                                                        |
|     v                                                                                                        |
|  +------+      +------------------+      +-----------------------------------+      +-------------+          |
|  | USB  | ---> | MCU              | ---> | DAC applies correction physically | ---> | 8-ch DAC    |          |
|  +------+      | RP2040 / ESP32   |      | as an analog voltage per physical |      |             |          |
|                |                  |      | p-bit, setting the Bernoulli p    |      |             |          |
|                |                  |      | requested by the MCU              |      +------+------+          |
|                | computes p from  |      +-----------------------------------+      | | | | | | | |          |
|                | J, h and states  |                                                 v v v v v v v v          |
|                +--------^---------+                                                VBIAS0 ... VBIAS7         |
|                         |                                                               |                    |
|                         |                                                               | to physical p-bits |
|                         |                                                               v                    |
|                         |                                             +----------------------------------+   |
|                         +---------------- Q0 ... Q7 ------------------| PHYSICAL P-BIT SECTION           |   |
|                                                                       +----------------------------------+   |
|                                                                                                              |
+--------------------------------------------------------------------------------------------------------------+
```
The DAC should have at least 12-bit resolution, and optionally it could use a 2.5 V precision reference. An isolated power supply could also be helpful, but it is not critical for the first prototype. Setting the p value for the Bernoulli distribution is achieved by applying the calibrated VBIAS correction:
```
p target
   ↓
MCU calibration mapping
   ↓
corrected VBIAS value
   ↓
DAC digital code
   ↓
analog VBIAS voltage
```
The correction is applied using a per-p-bit look-up table that converts the desired p value into the corrected VBIAS voltage output by the DAC. Next is the p-bits diagram.
```
+--------------------------------------------------------------------------------------------------+
|                                        PHYSICAL P-BIT SECTION                                    |
|                                                                                                  |
|  VBIAS0 ---> +--------------------+                     VBIAS7 ---> +---------------------+      |
|              |     [P-BIT 0]      |                                 |     [P-BIT 7]       |      |
|              |                    |                                 |                     |      |
|              |    Noise source    |                                 |    Noise source     |      |
|              |          ↓         |                                 |         ↓           |      |
|              |      Amplifier     |           . . .                 |     Amplifier       |      |
|              |          ↓         |                                 |         ↓           |      |
|              | Comparator(VBIAS0) |                                 | Comparator(VBIAS7)  |      |
|              |          ↓         |                                 |         ↓           |      |
|              |         Q0         |                                 |         Q7          |      |
|              +--------------------+                                 +---------------------+      |
|                     |                                                         |                  |
|                     +------------------- Q0 ... Q7 ---------------------------+------> MCU inputs|
|                                                                                                  |
|                       ... additional physical p-bits follow the same structure ...               |
+--------------------------------------------------------------------------------------------------+
```
The actual p-bit can be implemented with simple components such as capacitors, resistors, a physical noise source, an amplifier, and a comparator.
We have:
* Noise source: creates the random analog fluctuations.
* VBIAS: the control voltage that shifts the comparator threshold and therefore changes the probability that the p-bit outputs 1 or 0.

Q0 ... Q7 are simply the current 0/1 outputs of the eight physical p-bits. Together, these eight bits form one complete state of the probabilistic circuit—for example, `10100110`.
Because the p-bits keep changing, the MCU reads many such states over time. Some states appear more often than others.

Here is the main recursive loop:

```text
J, h + current Q0 ... Q7
          ↓
MCU computes target p
for each selected p-bit
          ↓
calibration correction
p → corrected VBIAS
          ↓
physical p-bit produces
new Q = 0 or 1
          ↓
Q0 ... Q7
          ↓
one 8-bit joint state
          ↓
many states collected over time
          ↓
joint distribution of the circuit
```

`J` and `h` are provided by the `PCircuit` we want to explore. The MCU sends the sampled joint states one by one back to p-kit. It is then p-kit that constructs or estimates the final joint distribution of the circuit from these samples. As this is mixed digital/analog processing, the MCU speed becomes important.

## Status

There are two main versions of the firmware:
* calibration_run.ino: executes circuits of up to eight p-bits using the eight physical p-bits and supports constant and linear annealing through probana_backend.py.
* calibrate_run_full_anneal_banking.ino: executes larger virtual circuits through banking and supports constant, linear, and arbitrary p-kit annealing schedules through probana_compile_backend.py.

## Future

* Controlled p-bit correlation through J
* Implementing J physically/analogically
