# QUICK REFERENCE: Adding Actions at End of Paths in Pedro Pathing

## The Pattern (Approach 1)

```java
case STATE_FOLLOW_PATH:
    follower.followPath(myPath, speed, true);
    setPathState(STATE_WAIT_FOR_PATH);
    break;

case STATE_WAIT_FOR_PATH:
    if (!follower.isBusy()) {  // ← Path complete!
        setPathState(STATE_DO_ACTION);
    }
    break;

case STATE_DO_ACTION:
    if (actionTimer.getElapsedTime() > 500) {  // ← Wait 500ms
        myMechanism.activate();  // ← Execute action
        setPathState(STATE_WAIT_FOR_ACTION);
    }
    break;

case STATE_WAIT_FOR_ACTION:
    if (actionTimer.getElapsedTime() > 1500) {  // ← Wait 1500ms total
        follower.followPath(nextPath, speed, true);  // ← Continue
        setPathState(NEXT_STATE);
    }
    break;
```

## Key Methods

| Method | Purpose |
|--------|---------|
| `follower.followPath(path, speed, true)` | Start following a path |
| `!follower.isBusy()` | Check if path is complete |
| `actionTimer.getElapsedTime()` | Get milliseconds since state started |
| `setPathState(newState)` | Transition state AND reset timer |

## Timing Formulas

```
Case 1: Simple delay before action
  if (actionTimer.getElapsedTime() > 500) {
    // Executes after 500ms from setPathState()
  }

Case 2: Action that takes time
  state 1: if (actionTimer.getElapsedTime() > 100) action.start()
  state 2: if (actionTimer.getElapsedTime() > 1100) { /* total 1000ms */ }

Case 3: Multiple actions in sequence
  state 1: if (actionTimer.getElapsedTime() > 100) action1.start()
  state 2: if (actionTimer.getElapsedTime() > 600) action2.start()
  state 3: if (actionTimer.getElapsedTime() > 1100) continue()
```

## Examples by Mechanism

### Launch After Path
```java
case 10:
    if (!follower.isBusy()) {
        setPathState(11);
    }
    break;

case 11:
    if (actionTimer.getElapsedTime() > 200) {
        launcher.launch();  // Execute
        setPathState(12);
    }
    break;

case 12:
    if (actionTimer.getElapsedTime() > 1700) {  // 1500ms launch time
        follower.followPath(nextPath, 1, true);
        setPathState(13);
    }
    break;
```

### Grab After Path
```java
case 20:
    if (!follower.isBusy()) {
        setPathState(21);
    }
    break;

case 21:
    if (actionTimer.getElapsedTime() > 100) {
        claw.close();  // Execute
        setPathState(22);
    }
    break;

case 22:
    if (actionTimer.getElapsedTime() > 600) {  // 500ms close time
        follower.followPath(nextPath, 1, true);
        setPathState(23);
    }
    break;
```

### Extend/Retract After Path
```java
case 30:
    if (!follower.isBusy()) {
        setPathState(31);
    }
    break;

case 31:
    if (actionTimer.getElapsedTime() > 100) {
        arm.extend();  // Execute
        setPathState(32);
    }
    break;

case 32:
    if (actionTimer.getElapsedTime() > 800) {  // 700ms extend time
        follower.followPath(nextPath, 1, true);
        setPathState(33);
    }
    break;
```

## State Naming Convention

**Recommended naming for readability:**

```java
// Main paths
case 0:   // Follow path 1
case 1:   // Wait for path 1
case 2:   // Follow path 2
case 3:   // Wait for path 2

// Actions at end of paths (offset by +50)
case 50:  // Do action after path
case 51:  // Wait for action

case 100: // Do next action
case 101: // Wait for next action

// Or use prefixes
case 0:   // SCORE_FOLLOW
case 1:   // SCORE_WAIT
case 2:   // SCORE_ACTION
case 3:   // SCORE_WAIT_ACTION

case 10:  // PICKUP_FOLLOW
case 11:  // PICKUP_WAIT
case 12:  // PICKUP_ACTION
case 13:  // PICKUP_WAIT_ACTION
```

## Testing Checklist

- [ ] Robot follows path correctly
- [ ] Action executes after path is complete
- [ ] Action duration is correct
- [ ] Next path starts after action completes
- [ ] Telemetry shows correct state transitions
- [ ] No conflicts between path and action

## Common Mistakes

❌ **Wrong: Execute action while path running**
```java
case 1:
    if (!follower.isBusy()) {
        follower.followPath(nextPath, 1, true);  // ← Don't do next path yet!
        mechanism.activate();
        setPathState(2);
    }
```

✅ **Right: Use intermediate state**
```java
case 1:
    if (!follower.isBusy()) {
        setPathState(2);  // ← Go to action state first
    }
    break;

case 2:
    if (actionTimer.getElapsedTime() > 500) {
        mechanism.activate();
        setPathState(3);
    }
    break;

case 3:
    if (actionTimer.getElapsedTime() > actionDuration) {
        follower.followPath(nextPath, 1, true);
        setPathState(4);
    }
    break;
```

❌ **Wrong: Timer not resetting**
```java
case 2:
    if (actionTimer.getElapsedTime() > 500) {
        mechanism.activate();
        pathState = 3;  // ← Timer doesn't reset! Bug!
    }
    break;
```

✅ **Right: Use setPathState()**
```java
case 2:
    if (actionTimer.getElapsedTime() > 500) {
        mechanism.activate();
        setPathState(3);  // ← Timer resets automatically
    }
    break;
```

## Debug Telemetry

Add this to your `loop()` method:

```java
@Override
public void loop() {
    follower.update();
    autonomousPathUpdate();
    
    // Debug
    telemetry.addData("Path State", pathState);
    telemetry.addData("Timer (ms)", actionTimer.getElapsedTime());
    telemetry.addData("Is Busy", follower.isBusy());
    telemetry.addData("X", follower.getPose().getX());
    telemetry.addData("Y", follower.getPose().getY());
    telemetry.update();
}
```

## Summary

1. **Path Complete?** → `if (!follower.isBusy())`
2. **Move to Action State** → `setPathState(actionState)`
3. **Wait then Execute** → `if (actionTimer > delay) { action() }`
4. **Wait for Completion** → `if (actionTimer > totalTime) { continue() }`
5. **Reset Timer Auto** → `setPathState()` does it

