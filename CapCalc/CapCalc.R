# dt: Time incremental value (Second)
# mt: Simulation until mt (Second)
# c: Capacitance (Farad)

t = dt
vi = 0
v = array(0)
tn = array(0)
n = 1
while(t < mt) {
  if(t %% (500 * 0.000001) < 0.000400) {
    vi <- 5
  } else {
    vi <- 0
  }
  v[n + 1] = (v[n] - vi) * exp(-dt/(1000000 * c)) + vi
  tn[n + 1] = t
  t = t + dt
  n = n + 1
}

# tn: Timestamp (Second)
# v: TP Voltage (Volt)