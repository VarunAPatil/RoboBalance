# Phase 1 CartPole result

Date: 2026-09-25. A CPU PPO run used seed 42 and the parameters in `configs/ppo.yaml`. The requested 100,000 timesteps became 100,352 actual timesteps because PPO completed its final 1,024-step rollout. The run is in `experiments/cartpole-ppo-seed42-100k/`.

The saved final policy scored 500.0 mean reward with 0.0 standard deviation on 30 independent CartPole episodes with seeds 10042–10071, the maximum allowed by CartPole-v1. A second invocation of `training.evaluate` on seeds 20042–20071 also scored 500.0 ± 0.0. `final_evaluation.json` and `holdout_evaluation_20042.json` contain episode-level results. The earlier 2,048-step smoke run scored 95.5 mean reward over 30 episodes; it checked that the training and saving path worked.

The training run saved TensorBoard scalars for reward, episode length, evaluation, and PPO optimization, plus monitor CSVs, checkpoints, versions, and a frozen dependency list. Run `tensorboard --logdir experiments` from the project root to inspect its curves.

This completes the first RL learning milestone. CartPole has discrete left/right actions and simplified dynamics, so this result is evidence for the software workflow only. It is not evidence that the robot model or deployed balancing policy will work.
