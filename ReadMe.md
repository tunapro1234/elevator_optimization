# Elevator Optimization

This project was developed for the Boğaziçi AI UpForAChallenge competition. It implements and compares elevator control algorithms within a comprehensive simulation environment.

*Note: This README is not the official paper. The full technical paper is available in the repository as `boaiUpforaChallengePaper.pdf`.*

## Project Overview

In this study, we developed and implemented two distinct elevator control algorithms:
1.  **Neuroevolution-based Approach**: Utilizes a neural network with three hidden layers to optimize elevator dispatching.
2.  **Simple Elevator Control Algorithm**: Based on deterministic dispatching strategies.

Both algorithms were integrated into a simulation environment designed to model realistic elevator dynamics and passenger behaviors. The system evaluates performance based on key metrics including:
*   Average Response Time
*   Average Waiting Time
*   Energy Consumption
*   System Throughput
*   Efficiency (weighted sum of normalized metrics)

## Current Status

We began strict coding on October 23rd, which limited the implementation of several planned features. The code is currently in a "post-hackathon" state.
*   The repository includes the implementation of the simulation and control algorithms.
*   Due to extensive computational requirements, the training of the Neuroevolution-based algorithm is incomplete.
*   Future work involves completing the neural network training, refining control algorithms, and implementing asynchronous structures (likely for PID loops).

## Authors' Notes

**About Us**
*   **Tuna**: 1st-year Civil Engineering student at Boğaziçi University.
*   **Yiğit**: 10th-grade student (future Boğaziçi University student, hopefully).

**Development Journey**
This project represents Yiğit's first experience with Rust and is his largest software project to date. As Tuna typically works alone, we faced some synchronization challenges during development. Consequently, the contribution graph and code structure may appear somewhat sporadic due to our different working hours.

**Technical Pivot**
We encountered compatibility issues with PyTorch Rust bindings (specifically involving Portage and LTO configurations). Consequently, instead of using standard bindings, we opted to write the neural network implementation from scratch. While challenging, this decision allowed us to bypass the dependency issues.

**Funding & Opportunities**
If you appreciate our code and have potential work opportunities, please reach out. We are currently raising funds to acquire a CNC router for our company, [NFR Products](https://nfrproducts.com).
*   *Note regarding deadlines*: We generally recommend setting an earlier internal deadline, as we occasionally face challenges with time management.

**Closing Thoughts**
We missed the intensity of day-and-night coding sessions. While the process was exhausting, it was an incredibly fun few days. If this repository gains significant traction, we commit to refactoring the codebase and updating this documentation to a higher standard. For now, it stands as a testament to our hackathon effort.
