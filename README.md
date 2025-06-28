# Predictive AODV Protocol with Link Quality Awareness – NS-3

## Overview

This project extends the standard AODV routing protocol in NS-3 by integrating **link quality prediction** and **probabilistic threshold-based forwarding**. The objective is to enhance the **resilience, stability, and efficiency** of routing in dynamic wireless networks by making smarter forwarding decisions based on predicted link reliability.

## Key Contributions

- 📡 **Link Quality Prediction**  
  Each routing decision considers predicted link quality between nodes, helping to avoid unstable or unreliable routes.

- 🎯 **Probabilistic Forwarding Threshold**  
  A probability-based threshold is used to determine whether a node should forward a route request (RREQ), helping reduce unnecessary transmissions and improve scalability.

- ⚙️ **Protocol Modifications**  
  Core AODV files were modified:
  - `aodv-routing-protocol.cc`
  - `aodv-routing-protocol.h`
  - Additional support changes across relevant classes for compatibility and logic control.

- 📈 **Simulation and Evaluation**  
  A custom experiment class `RoutingExperiment::Run()` manages the simulation workflow and collects metrics such as delivery ratio, average delay, and control overhead to evaluate protocol performance.

## Why This Matters

This modified protocol supports:
- **Improved reliability** under changing network conditions
- **Reduced congestion** through intelligent suppression of low-quality paths
- **Enhanced resilience** for networks in adversarial or high-mobility environments

The work reflects current research trends in **cyber-resilient routing**, **adaptive systems**, and **cross-layer optimization**.

## How to Use

1. Clone or download this repository into your NS-3 source directory.
2. Rebuild NS-3:
   ```bash
   ./waf configure
   ./waf build
