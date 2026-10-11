golden 回放对比 (在小电脑仓库根目录运行)

1. bash tests/golden/run_golden.sh <label>
   用当前代码回放 logs.txt 里的日志, bullet/laser 两种模式, 结果在 /home/seven/RM27_zimiao/golden/<label>/
2. diff -r /home/seven/RM27_zimiao/golden/baseline /home/seven/RM27_zimiao/golden/<label>
   纯精简(不改效果)的改动: 必须为空, 逐字节一致才能提交。
3. python3 tests/golden/metrics.py <dirA> <dirB> [bullet|laser]
   改效果的改动: 按工况(静止/慢陀螺/高速陀螺/纯平移/边转边移)对比开火率。
回放盲区: 前哨站/基地/大装甲板, 云台过冲(回放里云台 = 30ms 前的指令)。
