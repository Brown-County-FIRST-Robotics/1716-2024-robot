# PeriodicRunnable class
## Decision flowchart
```mermaid
flowchart TB
    q1(Does ONE subsystem depend on/use this?) -- Yes --> p1(PeriodicRunnable)
    q1 -- No --> q2(Do multiple commands depend on this, or is it used outside of a command?)
    q2 -- No --> p2(PeriodicRunnable)
    q2 -- Yes --> s1(Subsystem)
```

## Code example
```java
class Example extends PeriodicRunnable{
  public Example(){
    super(); // Super call adds it to the registry, which calls the periodic method every tick
  }
  @Override
  public void periodic(){
    //Periodic functionality
  }
}
```