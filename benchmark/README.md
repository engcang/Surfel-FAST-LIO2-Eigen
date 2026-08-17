# Benchmark summary

The checked-in figure is generated from `summary.csv` with:

```bash
python3 -m pip install matplotlib
python3 benchmark/plot_summary.py
```

The comparison contains Newer College Dataset (`01_short`, `02_long`, and
`05_quad`), 2021 HILTI (`Basement_1`, `Construction_Site_2`, and
`uzh_tracking_area_run2`), and NTU VIRAL (`spms_01`, `eee_02`, `nya_03`,
`sbs_02`, `tnp_02`, and `rtp_02`). Each implementation ran each sequence three
times with one-times-speed bag playback.

Translation APE uses EVO rigid SE(3) alignment without scale correction. The
APE value in `summary.csv` is the macro mean over successful runs; both methods
completed 34 of 36 runs under the common coverage and divergence criteria.
Mean scan time pools every valid scan. CPU 100% means one logical CPU core.

FAST-LIO2 Original ran as a ROS1 Noetic process in the prepared container,
whereas the Surfel implementation ran as a ROS2 Jazzy process on the host.
Dataset, input messages, playback rate, and CPU affinity were held constant,
but the middleware and dependency versions were not identical. These results
characterize the tested machine and configurations and are not universal
performance guarantees.
