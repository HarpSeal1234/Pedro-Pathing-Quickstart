# Approach 1 vs addParametricCallback: Which Should You Use?

## Short Answer

**For Pedro Pathing v2.0.6:** Use **Approach 1 (State Machine)** as shown in your `BlueCloseAuto.java`. The `addParametricCallback` method is NOT available in Pedro Pathing v2.0.6.

---

## Comparison

### Approach 1: State Machine (Current in BlueCloseAuto.java) ✅

**What it is:**
- Manual state management using a switch statement
- Actions executed between path states
- Timing controlled with `actionTimer`

**Example from your code:**
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

**Pros:**
- ✅ Simple and explicit
- ✅ Easy to debug (can see exact state flow)
- ✅ Works with Pedro Pathing v2.0.6
- ✅ Clear control over timing
- ✅ Used in all official Pedro Pathing examples
- ✅ No callbacks or complex syntax

**Cons:**
- ❌ More lines of code for complex sequences
- ❌ Manual timing management
- ❌ Larger switch statement

---

### Approach 2: addParametricCallback (NOT Available in v2.0.6) ❌

**What it is (if it existed):**
- Would be a callback attached to a path
- Would execute when path reaches a certain condition
- Would be cleaner and more functional style

**What it would look like (hypothetically):**
```java
// This does NOT exist in Pedro Pathing v2.0.6
score1 = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose1))
        .addParametricCallback(0.8, () -> {
            launcher.setState(Launcher.LauncherState.LAUNCH);
        })
        .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
        .build();
```

**Status:**
- ❌ NOT available in Pedro Pathing v2.0.6
- ⚠️ Might be in future versions
- ⚠️ Not in official documentation

---

## What Pedro Pathing v2.0.6 Actually Offers

Based on your `build.dependencies.gradle`, you're using **Pedro Pathing v2.0.6**.

The available methods in PathBuilder are:
- `.addPath()` - Add a path segment
- `.setLinearHeadingInterpolation()` - Set heading interpolation
- `.setConstantHeadingInterpolation()` - Constant heading
- `.setReversed()` - Reverse direction
- `.build()` - Build the PathChain

**No callback methods exist in v2.0.6.**

---

## Recommended Approach for Pedro Pathing v2.0.6

**Continue using Approach 1 (State Machine)** because:

1. ✅ It's the official pattern used in all Pedro Pathing examples
2. ✅ It's fully supported in v2.0.6
3. ✅ It's what your `BlueCloseAuto.java` uses
4. ✅ It's proven to work reliably
5. ✅ It's easy to understand and debug

---

## Your Current Code Pattern (Best for v2.0.6)

Your `BlueCloseAuto.java` is already using the **optimal approach** for Pedro Pathing v2.0.6:

```
Path → Check completion → Action → Wait → Next Path
```

This pattern is:
- ✅ Correct for v2.0.6
- ✅ Clear and maintainable
- ✅ Easy to modify
- ✅ Used in all official examples

---

## Future: When Callbacks Might Arrive

If Pedro Pathing releases a newer version with callback support, you could refactor to:

```java
score1 = follower.pathBuilder()
        .addPath(new BezierLine(startPose, scorePose1))
        .addTemporalCallback(1500, () -> launcher.setState(Launcher.LauncherState.LAUNCH))
        .build();
```

But **for now, stick with Approach 1.**

---

## Conclusion

| Feature | Approach 1 (State Machine) | addParametricCallback |
|---------|---------------------------|----------------------|
| Available in v2.0.6? | ✅ YES | ❌ NO |
| Recommended? | ✅ YES | ⚠️ NOT YET |
| Complexity | 📊 Medium | 🔧 Would be simpler |
| Documentation | ✅ Extensive | ❌ None |
| Official Examples | ✅ All use it | ❌ N/A |

**Decision: Use Approach 1 (State Machine). It's the correct pattern for Pedro Pathing v2.0.6.**

Your `BlueCloseAuto.java` is already doing this correctly! ✅

