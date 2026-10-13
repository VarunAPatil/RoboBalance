# Wall-E RL reading path

This project is also a way to learn reinforcement learning. Read in the order below; you do **not** need to understand every equation before running the CartPole experiment. Each resource has a specific question to answer. Original papers are linked at arXiv; optional local PDF reading copies are excluded from Git.

## Start now: the agent–environment loop

1. Read our [CartPole and Gymnasium walkthrough](../docs/CARTPOLE_GYMNASIUM.md) and [RL fundamentals](../docs/FUNDAMENTALS.md), then [OpenAI Spinning Up: Key Concepts in RL](https://spinningup.openai.com/en/latest/spinningup/rl_intro.html). Focus on **observation, action, reward, episode, policy, and return**. After reading, explain CartPole in one sentence for each term. On Wall-E, replace the cart push with a motor command and the pole angle with body pitch.
2. Read [Gymnasium's basic usage guide](https://gymnasium.farama.org/introduction/basic_usage/) and [CartPole description](https://gymnasium.farama.org/environments/classic_control/cart_pole/). Then inspect `training/common.py`: find `reset`, `step`, `reward`, `terminated`, and `truncated`. The environment is the thing being controlled; Gymnasium is the shared Python interface for talking to it.
3. Read [Stable-Baselines3's PPO page](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html) alongside `configs/ppo.yaml` and `training/train.py`. Identify `n_steps`, `gamma`, `batch_size`, and `clip_range`. At this stage, learn what each setting changes rather than memorizing a recommended value. [SB3's RL tips](https://stable-baselines3.readthedocs.io/en/master/guide/rl_tips.html) explain why we evaluate on separate episodes.

For a deeper textbook, [Sutton and Barto, *Reinforcement Learning: An Introduction*, second edition](https://mitpress.mit.edu/9780262039246/reinforcement-learning/) is the standard foundation. Start with Chapters 1 and 3 for the agent–environment framework and Markov decision processes. The book covers many methods we are not using yet; there is no need to read it cover to cover before building Wall-E.

**Checkpoint:** With the saved CartPole run, you should be able to point to (a) the observations, (b) the two possible actions, (c) the reward, (d) when an episode ends, and (e) the file that holds the trained policy. If any of those are unclear, we should work through one episode together before moving to the robot simulator.

## Next: how PPO learns

- [**Proximal Policy Optimization Algorithms**](https://arxiv.org/abs/1707.06347), Schulman *et al.*, 2017. Read the abstract, introduction, and the overall algorithm first. The central idea for us is that PPO gathers experience with its current policy and limits the size of the next policy change. Return to the mathematical objective once terms such as policy probability and advantage are familiar. The paper is not an ESP32 implementation guide.
- [OpenAI Spinning Up's PPO explanation](https://spinningup.openai.com/en/latest/algorithms/ppo.html) is a bridge between the conceptual guide and the paper's equations. Its sample implementation differs from Stable-Baselines3, so use it to understand the algorithm rather than to copy settings into our code.
- [**High-Dimensional Continuous Control Using Generalized Advantage Estimation**](https://arxiv.org/abs/1506.02438), Schulman *et al.*, 2015. Save this for when you ask why PPO uses a critic and what `gae_lambda` means. **Advantage** estimates whether an action worked better than expected; GAE trades some bias for a less noisy training signal. You do not need this paper to run or interpret the first CartPole experiment.

**Checkpoint:** Explain in your own words why one unlucky or lucky episode should not cause a huge change to the motor policy. That motivates PPO's limited updates and evaluation over many episodes.

## Later: MuJoCo and transfer to the physical robot

- [MuJoCo's modeling guide](https://mujoco.readthedocs.io/en/latest/modeling.html) explains the MJCF model format we will use after measuring the robot. Read it when we begin Phase 3; it is a physics-engine manual, not an RL tutorial.
- [**Sim-to-Real Transfer of Robotic Control with Dynamics Randomization**](https://arxiv.org/abs/1710.06537), Peng *et al.*, 2017. Read when we introduce uncertain mass, friction, motor strength, and delays. Its robot task differs from ours, so transfer the principle of training over plausible dynamics, not its exact parameters or claimed results.
- [MuJoCo's DC motor modeling note](https://mujoco.readthedocs.io/en/latest/_static/dcmotor.pdf) is advanced reference material for Phase 3. It relates voltage, speed, torque, gearing, friction, and saturation. We will first measure the TT motors and use only as much model detail as those measurements support.

**Checkpoint:** A policy that gets a high reward on one exact simulated robot can still fail when the real battery voltage or motor response differs. Before hardware deployment, we will evaluate on unseen randomized models, disturbances, and realistic actuator limits.

The explanations in this reading guide are project study notes. The linked papers retain their original authorship.
