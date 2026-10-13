# CartPole and Gymnasium, from zero

This guide explains the Phase 1 experiment before introducing the physical robot. Read it while looking at [`training/common.py`](../training/common.py), [`training/train.py`](../training/train.py), and [`configs/ppo.yaml`](../configs/ppo.yaml).

## The problem we are solving

Imagine a cart that can move left and right on a track. A pole is hinged to its top and starts almost upright. The hinge cannot directly drive the pole. The only control is to push the cart left or right. If the pole leans right, moving the cart right can put the base under it, but the amount and timing of the push matter. Move too far and the cart reaches the track limit. This is **CartPole**.

It is a small version of an inverted-pendulum control problem, which is why it is useful before working on Wall-E. The real robot also moves its support point to keep a body upright. CartPole has simpler physics and just two fixed-force actions; Wall-E has motors, wheels, friction, battery effects, noise, delay, and ultimately continuous PWM commands. Success in CartPole only proves that our learning workflow functions.

## What Gymnasium is

**Gymnasium is a Python interface and a collection of example problems.** It defines how a controller talks to a problem, whether the problem is CartPole, a MuJoCo robot, or a custom simulation we write. Think of `gym.make("CartPole-v1")` as constructing one instance of the CartPole simulator.

Gymnasium's essential calls are:

```python
env = gym.make("CartPole-v1")
observation, info = env.reset(seed=42)  # start a new attempt
observation, reward, terminated, truncated, info = env.step(action)
env.close()
```

Gymnasium does **not** decide the action and does **not** train a neural network. Stable-Baselines3 supplies the PPO training algorithm. MuJoCo will later supply detailed physics for our robot while our own Gymnasium environment provides the same `reset`/`step` interface. TensorBoard displays recorded training curves.

| Piece | Job in this project |
| --- | --- |
| Gymnasium | Standard `reset` and `step` interface for a control problem |
| CartPole | The built-in physics problem used for practice |
| Stable-Baselines3 PPO | Learns a policy by repeatedly using the environment |
| MuJoCo | Future physics engine for the measured two-wheel robot |
| TensorBoard | Viewer for reward and training graphs |

## What the controller sees and does

An **observation** is the information handed to the controller at one instant. CartPole gives four numbers, in this order:

| Index | Meaning | Unit | Why it matters |
| --- | --- | --- | --- |
| 0 | Cart position | metres | Keeps the cart within the track |
| 1 | Cart velocity | metres/second | Says which way the cart is moving and how fast |
| 2 | Pole angle from upright | radians | Says which way the pole is leaning |
| 3 | Pole angular velocity | radians/second | Says whether the pole is falling or recovering |

For example, `[0.1, 0.2, 0.05, -0.1]` means a cart right of center and moving right, with a pole angled to one side but rotating back. These are example numbers, not a state measured in our run. Knowing angle alone would be insufficient: the same angle could be getting worse quickly or returning upright. This is why angular velocity matters. For Wall-E, the MPU6050 can help estimate pitch and pitch rate, but the TT motors have no encoders to directly supply wheel position or speed.

An **action** is what the controller asks the world to do next. CartPole has `Discrete(2)`: `0` pushes left, `1` pushes right. The force magnitude is fixed by the environment; the controller chooses only the direction. This is quite different from Wall-E's proposed continuous motor action in `[-1, 1]`.

An **action space** declares which actions are legal. An **observation space** declares the shape and range of observations. They are interface contracts; they do not tell the controller which action is good.

## One step versus one episode

One **step** is a single action followed by one physics update. The sequence is:

```text
current observation → choose left or right → update cart/pole physics
                    → receive next observation, reward, and end flags
```

CartPole gives `+1` reward for each step, including the step on which failure occurs. The reward is a teaching signal: longer survival produces a larger total. The **episode return** is the sum of rewards in one attempt. A 20-step attempt scores 20; a 500-step attempt scores 500. CartPole ends an episode if the pole angle passes about ±12° or the cart position passes ±2.4 m. The `CartPole-v1` time limit is 500 steps.

An **episode** is a complete attempt from `reset()` until either end flag becomes true. `terminated=True` means a task-defined ending, such as falling or leaving the track. `truncated=True` means an external cutoff, here the time limit. In both cases the current attempt is over and the next one needs `reset()`. Keeping the two flags separate also matters to some learning algorithms when estimating future reward.

This is the loop in our [`evaluate()` function](../training/common.py): it calls `model.predict(observation, deterministic=True)`, passes the action to `env.step(action)`, adds the reward, and stops when `terminated or truncated`. During evaluation, model weights do not change.

## Where learning happens

A **policy** is a rule from observation to action. Our policy is a neural network. At first its weights are essentially untrained, so it usually fails quickly. PPO tries many episodes and updates the weights so actions that tend to produce better outcomes become more likely. During training it explores by sampling actions from its policy; during deterministic evaluation it selects its preferred action for the current observation.

PPO also trains a **critic**, which estimates how much future reward is likely from an observation. The critic helps decide whether a sampled action worked better than expected, called its **advantage**. PPO limits the size of policy updates to reduce abrupt changes from noisy experience. The critic and optimization machinery are only needed for training; future ESP32 inference will use the action-producing policy.

In [`training/train.py`](../training/train.py), `PPO(...)` constructs the learner and `model.learn(total_timesteps=...)` runs this cycle. A **training timestep** is one environment action/step, not a whole episode. Thus 100,000 timesteps can span hundreds of failed or successful episodes. `n_steps: 1024` in the config says PPO collects a block of 1,024 steps before a training update; the block can cross episode boundaries. The saved full run requested 100,000 steps and completed 100,352 because the final block was finished.

The program saves checkpoints while training, tests periodically in a separate environment, and then evaluates the final saved policy on fresh episodes. The final policy reached the maximum 500 reward in all 30 independent CartPole episodes recorded in [`PHASE1_RESULT.md`](PHASE1_RESULT.md). That establishes the CartPole software milestone, not the physical robot milestone.

## Live example: compare a fixed command with our trained PPO policy

For convenient terminal commands, run `.venv/bin/python -m training.demo --controller right` and `.venv/bin/python -m training.demo`. The PPO demo also prints actual action probabilities; add `--render` to watch it or `--stochastic` to sample actions. The [CartPole workflow PDF](CARTPOLE_WORKFLOW.pdf) gives the full hands-on sequence.

Run the example below from the project root in VS Code's terminal. It runs two complete attempts from exactly the same initial state:

1. A controller that always pushes right, ignoring the observation.
2. Our saved PPO policy, which chooses an action using all four observation numbers.

This is **inference and evaluation**: it loads an existing network and does not train it. We ran this example locally on 2026-09-26: always pushing right lasted 10 steps, while the saved policy lasted the full 500 steps. This one comparison illustrates the control loop; the separate 30-episode evaluations in [the Phase 1 result](PHASE1_RESULT.md) provide broader evidence.

Paste this whole command into the terminal:

```bash
.venv/bin/python - <<'PY'
from pathlib import Path

import gymnasium as gym
from stable_baselines3 import PPO

RENDER = False  # Change to True to watch both attempts in a simulation window.
checkpoint = Path("experiments/cartpole-ppo-seed42-100k/final_model.zip")
if not checkpoint.is_file():
    raise FileNotFoundError(f"Saved policy missing: {checkpoint}")
model = PPO.load(checkpoint, device="cpu")

def describe(observation):
    x, velocity, angle, angular_velocity = observation
    return (
        f"x={x:+.4f} m  v={velocity:+.4f} m/s  "
        f"angle={angle:+.4f} rad  omega={angular_velocity:+.4f} rad/s"
    )

for controller in ("always right", "trained PPO"):
    env = gym.make("CartPole-v1", render_mode="human" if RENDER else None)
    try:
        observation, info = env.reset(seed=20042)
        total_reward = 0.0
        print(f"\n{controller}: start {describe(observation)}")

        for step in range(1, 501):
            # Choose using the CURRENT observation, before advancing physics.
            if controller == "always right":
                action = 1
            else:
                predicted_action, _ = model.predict(observation, deterministic=True)
                action = int(predicted_action.item())

            observation, reward, terminated, truncated, info = env.step(action)
            total_reward += reward

            # These numbers describe the state AFTER applying the action.
            if step <= 5 or terminated or truncated:
                direction = "LEFT" if action == 0 else "RIGHT"
                print(
                    f"step={step:3d} action={direction:5s} {describe(observation)} "
                    f"reward={reward:.1f} terminated={terminated} truncated={truncated}"
                )

            if terminated or truncated:
                print(f"Finished: {step} steps, total reward {total_reward:.1f}")
                break
    finally:
        env.close()
PY
```

With `RENDER = False`, it prints the trace quickly without opening a window. With `RENDER = True`, Gymnasium displays the cart and pole as the loop runs. If the renderer reports that `pygame` is missing, install the classic-control rendering extra with `.venv/bin/python -m pip install 'gymnasium[classic-control]'`, then rerun. Run the command in a desktop terminal for the windowed version.

### Read the actual output

Here is the locally captured trace, shortened to show the important rows. The values are rounded to four decimal places; the controller receives the full observation. Small differences may occur with other dependency versions.

| Controller | Step | Action | Position (m) | Velocity (m/s) | Angle (rad) | Angular velocity (rad/s) | Reward | Terminated | Truncated |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| Both | Reset | — | +0.0266 | −0.0253 | +0.0061 | +0.0322 | — | — | — |
| Always right | 1 | Right | +0.0261 | +0.1697 | +0.0067 | −0.2586 | 1 | False | False |
| Always right | 2 | Right | +0.0295 | +0.3648 | +0.0015 | −0.5492 | 1 | False | False |
| Always right | 5 | Right | +0.0630 | +0.9506 | −0.0490 | −1.4378 | 1 | False | False |
| Always right | 10 | Right | +0.1973 | +1.9303 | −0.2561 | −3.0691 | 1 | True | False |
| Trained PPO | 1 | Left | +0.0261 | −0.2205 | +0.0067 | +0.3268 | 1 | False | False |
| Trained PPO | 2 | Right | +0.0216 | −0.0255 | +0.0133 | +0.0362 | 1 | False | False |
| Trained PPO | 3 | Left | +0.0211 | −0.2208 | +0.0140 | +0.3330 | 1 | False | False |
| Trained PPO | 4 | Right | +0.0167 | −0.0259 | +0.0206 | +0.0448 | 1 | False | False |
| Trained PPO | 5 | Left | +0.0162 | −0.2213 | +0.0215 | +0.3439 | 1 | False | False |
| Trained PPO | 500 | Right | +0.2902 | −0.0254 | +0.0028 | +0.0349 | 1 | False | True |

### What happens at reset?

`env.reset(seed=20042)` starts a new attempt and returns the initial observation. Reset itself does not earn a reward. The seed makes the initial random state repeatable with compatible versions, so both controllers face the same starting situation.

Read the initial state as follows:

- `x = +0.0266`: the cart is about 2.66 cm right of the center.
- `v = −0.0253`: it is moving left slowly.
- `angle = +0.0061`: the pole is tilted slightly toward the right, about 0.35 degrees from upright.
- `omega = +0.0322`: its angle is increasing toward the right.

These numbers come from the simulator's state, not from physical sensors. On Wall-E we must obtain the chosen observations through real sensing or a validated estimator.

### Why does pushing right change the pole's motion?

At step 1 of the fixed controller, the rightward force makes the cart's velocity positive. Moving the hinge affects the pole's rotation: its angular velocity changes from `+0.0322` to `−0.2586` rad/s. The pole is still tilted slightly right, but is now rotating toward the left.

Notice that the cart's position initially decreases slightly despite the rightward force. The installed CartPole implementation advances positions using the previous velocities over its 0.02-second timestep, then updates velocities from the acceleration. Initially the cart was moving left, so this is consistent with its update equations. A force changes velocity; it does not instantly relocate the cart.

The controller keeps pushing right even after the pole starts rotating left. By step 5 the cart is moving rapidly right and the pole is tilted left with a growing leftward angular velocity. At step 10 the pole angle is `−0.2561` rad, about `−14.67°`, beyond the roughly `±12°` failure limit. The cart position is still inside the track limits, so this episode ends because of the pole angle.

The failure step still gives `+1` in this environment's default reward scheme. Ten steps therefore give a total reward of 10. `terminated=True` tells us the task has ended; we must stop stepping and reset before another attempt.

### What is the trained network doing differently?

The saved actor receives the current four-number observation and chooses its preferred action. It chooses left first, then right, then left in the first few rows. This is an observed sequence, **not an alternating rule written into the controller**. On every step it recomputes the action from the new observation.

Look at PPO steps 1 and 2. After the left push, the cart velocity is `−0.2205` m/s. The right push brings it closer to zero, `−0.0255` m/s. Angular velocity similarly changes from `+0.3268` to `+0.0362` rad/s. The pole angle is still positive and has increased, but its rightward rotation is much slower after that second action. This is why looking only at angle misses important information about recovery.

We can observe these effects; the trace does not reveal a human-style explanation inside the neural network. Its weights were adjusted during earlier PPO training to favor actions associated with better future returns.

At step 500, the pole is almost upright and the cart is within the allowed region. The episode ends with `terminated=False, truncated=True`: it reached the time limit. Its total reward is 500. That means it survived this simulated attempt, not that it returned the cart exactly to center or that Wall-E can balance physically.

### Where are PPO's other blocks during this example?

```text
During this example:
observation → saved actor → action → CartPole → next observation and reward

During training in training/train.py:
collect experience → estimate advantages using critic predictions
                   → optimize PPO actor loss and critic loss
                   → collect fresh experience with updated weights
```

`model.predict(...)` runs the saved policy without updating its weights. There is no rollout-buffer training, advantage calculation, or optimizer update in this example. Calling `model.learn(...)` in the training script is what activates that learning cycle. The reward is printed and accumulated here to measure performance; merely receiving it does not make the network learn.

Try changing the seed to `20043` and rerunning both controllers. Predict which columns will change before looking at the output. Then identify a step where the pole angle is positive but its angular velocity is negative: it is tilted right while rotating back toward upright.

## Try a few steps yourself

From `wall-e-rl/`, with `.venv` activated, run this in Python. It uses fixed actions so you can see the interface without needing PPO:

```python
import gymnasium as gym

env = gym.make("CartPole-v1")
observation, info = env.reset(seed=42)
print("start:", observation)

for step_number in range(5):
    action = step_number % 2  # alternate left and right; no learning here
    observation, reward, terminated, truncated, info = env.step(action)
    print(step_number, action, observation, reward, terminated, truncated)
    if terminated or truncated:
        break

env.close()
```

Try to identify the pole angle and angular velocity in each printed observation. Then ask what information you would need to choose a *better* action than alternation. That question is the input-design problem we will revisit for Wall-E.

Official references: [Gymnasium basic usage](https://gymnasium.farama.org/introduction/basic_usage/), [CartPole specification](https://gymnasium.farama.org/environments/classic_control/cart_pole/), and the [Stable-Baselines3 PPO guide](https://stable-baselines3.readthedocs.io/en/master/modules/ppo.html).
