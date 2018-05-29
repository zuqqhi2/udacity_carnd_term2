#!/usr/bin/python3

import sys
import subprocess

# Execute 1 episode and get total error
def run(p):
  pid_cmd = './build/pid {} {} {} {} {} {}'.format(p[0], p[1], p[2], p[3], p[4], p[5])
  text_format_cmd = r"| grep 'CTE' | awk -F' ' '{ print $2 }'"

  # Run for 30 sec
  raw_errors = subprocess.check_output(r'/usr/bin/timeout 30 {} {}'.format(pid_cmd, text_format_cmd), shell=True, universal_newlines=True)

  # Convert error string to float array without 1st 50 data
  errors = [float(v) ** 2 for i, v in enumerate(raw_errors.split('\n')[:-2]) if i >= 50]

  return sum(errors) / float(len(errors))

# Use this instead of sum(dp) because each value's range is different
def calc_weighted_dp(dp):
  steer    = dp[0] * 100.0 / 2.0 + dp[1] * 10000.0 + dp[2] * 10.0
  throttle = dp[3] * 100.0 / 2.0 + dp[4] * 10000.0 + dp[5] * 10.0
  return steer + throttle


# Main
p   = [ 0.2,  0.004, 3.0,  0.2,  0.004, 3.0]
dp  = [0.01, 0.0001, 0.1, 0.01, 0.0001, 0.1]
tol = 0.2
throttle_flg = False

if len(sys.argv) >= 2:
  if sys.argv[1] == '--throttle':
    dp[0] = dp[1] = dp[2] = 0.0
    throttle_flg = True
else:
  dp[3] = dp[4] = dp[5] = 0.0

abs(run(p))
best_err = abs(run(p))
num_it = 0
while calc_weighted_dp(dp) > tol:
  print('=== Please reset simulator and type something after reset ===')
  input()

  print('Iteration {:>3}, best error = {:>.5}, sum dp = {:>.5}'.format(num_it, best_err, calc_weighted_dp(dp)))
  print('p : [{:>.5}, {:>.5}, {:>.5}], [{:>.5}, {:>.5}, {:>.5}]'.format(p[0], p[1], p[2], p[3], p[4], p[5]))
  print('dp: [{:>.5}, {:>.5}, {:>.5}], [{:>.5}, {:>.5}, {:>.5}]'.format(dp[0], dp[1], dp[2], dp[3], dp[4], dp[5]))
  for i in range(len(p)):
    p[i] += dp[i]
    err = abs(run(p))
    if err < best_err:
      best_err = err
      dp[i] *= 1.1
    else:
      p[i] -= 2.0 * dp[i]
      err = abs(run(p))
      if err < best_err:
        best_err = err
        dp[i] *= 1.1
      else:
        p[i] += dp[i]
        dp[i] *= 0.9
  num_it += 1

print('Best parameters: {}, {}, {}'.format(p[0], p[1], p[2]))
