Probana is a research probabilistic analog p-bit computer. It can be used as backend for [p-kit](https://github.com/IBM/p-kit).

## Objectives:

* develop a probabilistic computer, initially with 8 or 16 physical p-bits
* run p-kit probabilistic circuits
* provide a full implementation using inexpensive, off-the-shelf components
* USB pluggable to any computer
* provide firmware and PCB designs
* potentially be modular, so that several 8-p-bit boards can be joined together to easily form an 8 × N probabilistic computer

## How it works or Why it should work?

The trick is to perform a calibration in the beginning. We measure the analog representation of each p-bit and we apply a software adjustment. Each p-bit implements a Bernoulli distribution. This distribution has a parameter, `p`, which defines the probability of the p-bit being `1`.

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
