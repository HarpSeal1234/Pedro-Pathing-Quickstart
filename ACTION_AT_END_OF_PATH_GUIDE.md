# How to Add Actions at the End of a Path in Pedro Pathing

## Overview
In Pedro Pathing, actions at the end of paths are handled using a **state machine** approach within your autonomous OpMode. The most common and recommended approach is **Approach 1: State-Based Actions with Timer Delays**.

---

## Approach 1: State-Based Actions with Timer Delays (RECOMMENDED)

This is the approach used in the `BlueCloseAuto.java` and `BlueFarAuto.java` examples in your project.

### How It Works

1. **Follow a Path** - Robot follows a path to a specific destination
2. **Check if Path is Complete** - Use `!follower.isBusy()` to detect when the path is finished
3. **Execute Action** - Once the path is complete, execute your desired action
4. **Transition to Next State** - Move to the next state for the next path or action

### Key Components

#### 1. Define Your Paths
```java
private PathChain score1, pickup1;

public void buildPaths() {
    score1 = follower.pathBuilder()
            .addPath(new BezierLine(startPose, scorePose1))
            .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
            .build();
    
    pickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose1, pickup1Pose))
            .setLinearHeadingInterpolation(scorePose1.getHeading(), pickup1Pose.getHeading())
            .build();
}
```

#### 2. Create Timer for Action Timing
```java
private Timer actionTimer;

@Override
public void init() {
    actionTimer = new Timer();
    // ... other initialization ...
}
```

#### 3. Implement State Machine in autonomousPathUpdate()
```java
public void autonomousPathUpdate() {
    switch (pathState) {
        case 0: // Start following path
            follower.followPath(score1, 1, true);
            setPathState(1);
            break;
        
        case 1: // Wait for path to complete
            if (!follower.isBusy()) {
                // Path is done, execute action
                launcher.setState(Launcher.LauncherState.LAUNCH);
                setPathState(2);
            }
            break;
        
        case 2: // Wait for action to complete
            if (actionTimer.getElapsedTime() > 1500) {
                // Action is done, move to next path
                follower.followPath(pickup1, 0.8, true);
                setPathState(3);
            }
            break;
        
        case 3: // Wait for next path to complete
            if (!follower.isBusy()) {
                setPathState(-1); // Done
                requestOpModeStop();
            }
            break;
    }
}
```

#### 4. Manage State Transitions
```java
public void setPathState(int state) {
    pathState = state;
    pathTimer.resetTimer();
    actionTimer.resetTimer(); // Reset timer when changing states
}
```

---

## Real Example from BlueCloseAuto.java

Here's how the scoring at the end of a path is implemented:

```java
case 1: // Wait until arrived at score1, then LAUNCH
    if (!follower.isBusy()) {
        if (actionTimer.getElapsedTime() > 500) {
            launcher.setState(Launcher.LauncherState.LAUNCH);
            setPathState(101);
        }
    }
    break;

case 101: // Wait for launch to finish
    if (actionTimer.getElapsedTime() > launchTime) {
        follower.followPath(pickup1, 1, true);
        launcher.setState(Launcher.LauncherState.PICKUP);
        setPathState(2);
    }
    break;
```

**Breakdown:**
- **Case 1**: Robot waits for the path to complete (`!follower.isBusy()`)
- **Still in Case 1**: After a 500ms delay, it triggers the launcher
- **Case 101**: Robot waits for the launcher action to complete (1500ms)
- **Still in Case 101**: After action is done, it follows the next path and updates the launcher state

---

## Step-by-Step Guide for Your Code

### Step 1: Identify where you want the action
Determine which path completion should trigger your action.

### Step 2: Create an intermediate state
Create a "waiting" state that checks if the action should be triggered:

```java
case 10: // Path completed, wait for action timing
    if (actionTimer.getElapsedTime() > 500) { // 500ms after arrival
        myRobot.doSomething(); // Execute your action
        setPathState(11);
    }
    break;

case 11: // Wait for action to complete
    if (actionTimer.getElapsedTime() > actionDuration) {
        // Continue to next path
        follower.followPath(nextPath, speed, true);
        setPathState(12);
    }
    break;
```

### Step 3: Use actionTimer for timing
The `actionTimer` resets every time you call `setPathState()`, allowing you to:
- Delay action start: `if (actionTimer.getElapsedTime() > delayMs)`
- Wait for action completion: `if (actionTimer.getElapsedTime() > durationMs)`

---

## Common Actions at Path End

### 1. Activate a Mechanism
```java
case 5:
    if (!follower.isBusy()) {
        arm.setPosition(ARM_EXTENDED); // Extend arm
        setPathState(6);
    }
    break;

case 6:
    if (actionTimer.getElapsedTime() > 1000) { // Wait 1 second
        follower.followPath(nextPath, 1, true);
        setPathState(7);
    }
    break;
```

### 2. Launch/Score
```java
case 8:
    if (!follower.isBusy()) {
        launcher.setState(Launcher.LauncherState.LAUNCH);
        setPathState(9);
    }
    break;

case 9:
    if (actionTimer.getElapsedTime() > 2000) { // Wait for launch
        follower.followPath(nextPath, 1, true);
        launcher.setState(Launcher.LauncherState.PICKUP);
        setPathState(10);
    }
    break;
```

### 3. Grab/Pickup
```java
case 11:
    if (!follower.isBusy()) {
        claw.closeGrip();
        setPathState(12);
    }
    break;

case 12:
    if (actionTimer.getElapsedTime() > 500) { // Wait for grip to close
        follower.followPath(nextPath, 1, true);
        setPathState(13);
    }
    break;
```

---

## Best Practices

1. **Use Consistent State Numbering**
   - Use clear state numbers (0, 1, 2... for main path transitions)
   - Use intermediate states for actions (101, 102... or 1, 11, 12...)

2. **Reset Timer on State Change**
   - Always call `setPathState()` to reset the `actionTimer`
   - This ensures timing is consistent

3. **Check `!follower.isBusy()` before actions**
   - Never execute an action while the robot is still following a path
   - This prevents conflicts

4. **Use Appropriate Delays**
   - Adjust timing based on mechanism speed
   - Test timing to ensure smooth operation

5. **Add Telemetry for Debugging**
   ```java
   telemetry.addData("path state", pathState);
   telemetry.addData("action timer", actionTimer.getElapsedTime());
   telemetry.addData("is busy", follower.isBusy());
   telemetry.update();
   ```

---

## Summary

The state machine approach (Approach 1) is the standard way to add actions at the end of paths in Pedro Pathing:

1. ✅ Follow a path with `follower.followPath()`
2. ✅ Check completion with `!follower.isBusy()`
3. ✅ Execute action in the next state
4. ✅ Use `actionTimer` to control action timing
5. ✅ Transition to next path via `setPathState()`

This approach is flexible, easy to debug, and allows complex autonomous sequences with multiple actions and paths.

