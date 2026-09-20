# Git 日常操作与故障排查保姆级手册

适用对象：想自己完成保存、上传、下载、合并和排错，不想每一步都重新问人的你。
本文是操作说明，不是执行记录。以下仓库、分支、冲突和 SSH 配置来自历史对话，**本次写文档没有重新核验当前状态**。
旧文档《Git拉取更新与保留本地修改指南.md》保留不变；其中的提交编号和“这次已经做完”也只代表当时。

## 目录

1. [安全底线与仓库地图](#一安全底线与仓库地图)
2. [终端路径与 Git 的四层关系](#二终端路径与-git-的四层关系)
3. [先看状态，再判断同步方向](#三先看状态再判断同步方向)
4. [保存、暂存、提交、推送](#四保存暂存提交推送)
5. [下载更新与保护未提交修改](#五下载更新与保护未提交修改)
6. [分叉、冲突与 Gimbal 实例](#六分叉冲突与-gimbal-实例)
7. [差异、引号与 vi 编辑器](#七差异引号与-vi-编辑器)
8. [HTTPS、Clash 与令牌认证](#八httpsclash-与令牌认证)
9. [SSH 超时与 443 端口](#九ssh-超时与-443-端口)
10. [查克隆时间](#十查克隆时间)
11. [日常配方与错误决策树](#十一日常配方与错误决策树)
12. [最后检查清单](#十二最后检查清单)

## 一、安全底线与仓库地图

- 每次只运行一个命令块，看完输出再继续。不要把整篇当脚本粘贴。
- 先保存编辑器中的文件。重要记录、配置和冲突整理结果，另存一份到仓库外。
- 不强制推送，不用硬重置或批量清理来“解决”不明状态，不为追求 clean 删除自己的文件。
- 遇到路径、分支、提交范围不符合预期，立即停止写操作；可以继续查看状态和差异。
- 未完成 merge、rebase 或其他操作时，不叠加新的更新流程。本文主要讲 merge，不套用于 rebase。

| 工程 | 本地根目录，历史记录 | 分支 | origin，历史记录 |
| --- | --- | --- | --- |
| 自瞄 | `/Users/zy/RM27_zimiao/27zm_zy_20260831` | `master` | `https://github.com/zy-2006-08/27zm_zy_20260826.git` |
| 底盘 | `/Users/zy/Newcode/Chassisl_26_SHANGTAIJIE` | `master` | `https://github.com/zy-2006-08/Chassisl_26_SHANGTAIJIE.git` |
| 云台 | `/Users/zy/Newcode/Gimbal_26hero_CH010_20260725` | `master` | `https://github.com/zy-2006-08/Gimbal_26hero_CH010_20260725.git` |
| 终端 | `/Users/zy/RMGF/terminal` | `main` | `git@github.com:zy-2006-08/RM_Terminal_27.git` |

底盘、云台的 GitHub 仓库名与各自本地目录同名。自瞄本地日期与远端名字不同，不代表配置错了。
这四个是独立仓库。在其中一个目录执行提交，不会替另外三个保存。

## 二、终端路径与 Git 的四层关系
```bash
pwd
```
解释：显示终端当前目录。编辑器打开了某个工程，不代表终端也在那个工程里。
```bash
cd /Users/zy/RM27_zimiao/27zm_zy_20260831
```
解释：切到自瞄根目录。底盘、云台、终端操作时，把参数换成上表对应的完整路径。
```bash
cd "pages"
```
解释：进入当前目录下面的 `pages`，前提是它存在。这是相对路径，不是从磁盘根目录找；Git 日常操作建议回到仓库根目录做。
```bash
cd ..
```
解释：回到上一级目录。开头的 `/` 表示绝对路径的根，路径内部的 `/` 分隔目录；空格不会代替 `/`，反而会把参数拆开。

含空格路径要用成对英文引号包住，例如 `"/Users/zy/My Projects"`。输入部分目录名后按 Tab 可补全；中文引号 `“ ”` 不能代替英文引号。
```bash
git rev-parse --show-toplevel
```
解释：显示当前 Git 仓库根目录。若报 `not a git repository`，先检查位置，不要随手初始化一个新仓库。
```bash
git remote -v
```
解释：查看远端地址。`-v` 是 verbose，表示详细输出，不是版本号；通常分别显示 fetch 下载地址和 push 上传地址，检查两行是否都正确。

`origin` 是远端的本地别名，不等于 GitHub 本身。`master` 与 `main` 都只是分支名，不能混用，也没有谁天然更高级。
`origin/master` 是本地保存的“上次看到的远端 master”；推送命令里的 `origin master` 则是两个参数：目的远端、本地分支。

| 层次 | 像什么 | 改变它的典型动作 |
| --- | --- | --- |
| 工作区 worktree | 硬盘上正在编辑的文件 | 编辑器保存、合并、恢复 stash |
| 暂存区 index | 下一次提交的候选快照 | add 暂存文件当时的内容 |
| 本地提交 commit | 本机版本历史中的快照 | commit 保存整个暂存区 |
| 远端与远端跟踪引用 | GitHub 上的历史，以及本地记录的远端位置 | push 上传；fetch 刷新本地远端记录 |

保存文件不等于暂存；暂存不等于提交；提交不等于上传。暂存后再编辑，必须再次暂存，新内容才会进入提交。

## 三、先看状态，再判断同步方向
```bash
git status
```
解释：查看当前分支、暂存与未暂存修改、未跟踪文件，以及是否处于未完成操作。先看它，避免带着冲突继续更新。

| 提示 | 含义与下一步 |
| --- | --- |
| `working tree clean` | 没有待提交改动，但不保证已上传或远端没有更新 |
| `Changes not staged` | 已跟踪文件改了，尚未把这些改动放入暂存区 |
| `Changes to be committed` | 下次提交会包含这些暂存内容，要检查全部 |
| `Untracked files` | Git 尚未跟踪，默认 stash 不会收走 |
| `ahead` | 相对本地记录的上游，有本地提交待上传 |
| `behind` | 相对本地记录的上游，有远端提交待合入 |
| `diverged` | 双方各有对方没有的提交，不能直接快进 |
| `Unmerged paths` | 有冲突尚未标记处理完，先解决冲突 |
```bash
git fetch origin
```
解释：下载对象并更新 `.git` 中的远端跟踪信息，不直接更新工作区或把更新合入当前分支。失败就停；先成功 fetch，再看 status 的同步提示才更接近远端实际状态。

status 的 ahead/behind 依赖上游配置；没显示不代表双方相同。远端随时可能又变化，fetch 后的判断也只代表这次观察。
```bash
git log --oneline --left-right HEAD...origin/main
```
解释：终端仓库使用此命令比较双方独有提交。`HEAD` 是当前提交，三个点表示双方差集；`<` 是本地独有，`>` 是远端独有。三个 master 仓库改用 `origin/master`。
```bash
git log --oneline HEAD..origin/main
```
解释：两个点只显示远端有、本地没有的提交。没有输出仍可能是本地领先，不能当作双向同步证明；master 仓库替换分支名。
```bash
git log --oneline origin/master..HEAD
```
解释：在 master 仓库查看将要上传的本地独有提交。终端仓库改用 `origin/main`，核对是否夹带不想共享的提交。

成功命令可能没有输出。空输出不是“全部同步”的通行证，必须结合命令含义、退出是否成功和最新分支比较来判断。

## 四、保存、暂存、提交、推送

第一步：在编辑器保存目标文件，确认终端位于正确仓库、正确分支，且没有未完成操作。
```bash
git diff -- "学习记录.txt"
```
解释：检查该文件未暂存的改动。新建的未跟踪文件通常不在此差异中，要直接打开阅读。
```bash
git add -- "学习记录.txt"
```
解释：把指定文件此刻的内容暂存。`--` 后面按路径处理；不是提交，也不会替你保存编辑器中尚未落盘的文字。
```bash
git diff --cached
```
解释：查看所有暂存内容相对于当前提交的差异。不要只看一个目标文件，因为下一条 commit 默认保存整个暂存区。
```bash
git diff --cached --check
```
解释：检查暂存差异中的冲突标记和空白问题。残留冲突标记要严肃处理；尾部空格等通常是格式问题，Markdown 两个尾空格可能是有意换行，要按项目约定判断。
```bash
git commit -m "整理学习记录"
```
解释：把所有已暂存内容保存为本地提交。`-m` 是 message，后面是提交说明，不是文件过滤器。没有实际变化就不需要创建提交。
```bash
git push origin master
```
解释：将本地 master 中远端缺少的所有祖先提交推到 origin 的 master，不是只上传刚刚那一个；终端仓库必须改为 `git push origin main`，即上传本地 main 到远端 main。
```bash
git commit -am "更新已跟踪文件"
```
解释：这是理解用的替代方式，不是上一流程的下一步。`-a` 自动纳入已跟踪文件的修改与删除，`-m` 写说明；不会自动加入未跟踪的新文件，而且范围很广，也会包含原本已暂存内容，新手优先逐文件 add。

Git 记录文件，不记录空目录。想上传 `pages/`，要有真实文件，并确认不是忽略项；不要以为创建空文件夹就能提交。
```bash
git add -- "pages/index.html"
```
解释：只暂存确实存在且要共享的页面文件。不要为上传一个页面就盲目暂存整个仓库，也别顺手包含构建产物、令牌或本机配置。

## 五、下载更新与保护未提交修改

### 5.1 工作区干净：只允许快进
```bash
git merge --ff-only origin/master
```
解释：成功 fetch 且确认在 master 后使用。只允许沿已有历史前进，不创建分叉合并提交；终端仓库用 `origin/main`。`Fast-forward` 或 `Already up to date.` 都是成功，但后者不代表本地提交已经上传。

不能快进就转到第六节。出现“本地改动将被覆盖”或“未跟踪文件将被覆盖”就停，备份并处理那些文件，不要强行覆盖。

### 5.2 有修改：两种 stash 范围只选一个
```bash
git stash push -m "更新前保存学习记录" -- "学习记录.txt"
```
解释：方式 A，只收起指定已跟踪文件的未提交修改，其他文件不动；`-m` 是 stash 备注，方便之后辨认。这不是本地 commit，更不是外部备份。
```bash
git stash push -m "更新前保存全部已跟踪修改"
```
解释：方式 B，替代 A，收起全部已跟踪文件的已暂存与未暂存修改。范围更大，确认确实都要暂时收起才用，不要把 A、B 连着执行。
```bash
git stash push -u -m "更新前保存含未跟踪文件的修改"
```
解释：只有确实需要时，使用这个替代方案。`-u` 还会收起未跟踪文件，使它们暂时从工作区消失，但不包含被忽略文件；先确认范围并做外部备份，不与上面两种叠加运行。

看到 `Saved working directory and index state` 才确认保存成功。`No local changes to save` 表示没有新 stash，后面不能拿旧 stash 冒充本次备份恢复。
先看 status，确认剩余改动不会阻挡更新，再执行 5.1。定向 stash 后还有其他改动是可能的；发生任何更新错误，先停，不继续 apply。
```bash
git stash list
```
解释：列出 stash 编号和备注，找到本次确实新建的那份。编号会随新增或删除发生变化，不要凭记忆选。
```bash
git stash show -p --include-untracked 'stash@{0}'
```
解释：查看这份 stash 的补丁，包括其中保存的未跟踪内容，确认属于本次操作。`0` 只是示例，必要时替换为核实后的编号；内容可能含隐私，不要随便截图外发。
```bash
git stash apply 'stash@{0}'
```
解释：只有更新成功、目标 stash 已核实才恢复。apply 保留 stash 备份，默认不保证恢复原来的暂存分组；随后重新检查、暂存。出现冲突也可能已恢复部分内容，**不要重复 apply**。

没有本次新 stash 就跳过恢复。stash 冲突按第六节编辑、保存、暂存，但它通常不是正在进行的 merge，不能用 merge abort 当作撤销 stash 的方法。

## 六、分叉、冲突与 Gimbal 实例

### 6.1 双方都有提交：普通合并，不强推

确认已备份、工作区与暂存区干净，fetch 成功，并看过双方独有提交后再开始。干净起步更容易安全撤销。
```bash
git merge --no-edit origin/master
```
解释：把远端跟踪分支合入当前 master，接受默认合并说明。普通合并默认能快进就快进，分叉时创建合并提交；仓库配置可能改变默认行为。终端仓库改成 `origin/main`。
```bash
git merge -m "合并远端 master 更新" origin/master
```
解释：这是上一条的替代写法，自定义合并说明，不要两条都跑。`-m` 不会强制产生合并提交，也不能跳过冲突；发生冲突时先处理文件。

### 6.2 找全冲突，逐个决定内容
```bash
git -c core.quotePath=false diff --name-only --diff-filter=U
```
解释：列出所有仍处于 unmerged 状态的文件；临时关闭中文路径转义，方便认名字，不修改永久配置。没有输出只说明没有 U 项，不保证代码正确或标记已清理。

普通 merge 的冲突块里，`<<<<<<< HEAD` 一侧是合并开始时本地当前分支内容；另一侧通常是传入的 `origin/master` 或 `origin/main`。
这个解释只适用于这里的 merge。stash 常显示 `Updated upstream` 与 `Stashed changes`，rebase 的视角也不同，不要一概套用“上面永远是我的”。
编辑器的“接受双方”只是拼接，可能制造重复函数、重复配置、互相矛盾的步骤或无效代码，不是万能正确答案。

| 冲突类型 | 实际要决定什么 |
| --- | --- |
| content | 两边改了同一处，读懂后保留一边或重新整理成正确结果 |
| add/add | 两边分别新增同名文件，比较用途和内容，不能假定任一边完整 |
| modify/delete | 一边删除、一边修改，明确文件现在是否还需要 |

历史上 Gimbal 曾出现下面七个冲突，**不是本次重新检查得到的当前列表**：
1. `Core/Src/my_main.cpp`，核对控制流程、接口和重复定义。
2. `Mac迁移配置指南.md`，合并仍然适用的步骤，去掉互相矛盾的描述。
3. `ozone/yuntai.jdebug`，核对调试工程路径与目标配置。
4. `ozone/yuntai.jdebug.user`，先决定是否保留这个用户配置文件。
5. `scripts/uart_read_win.py`，检查逻辑是否完整，不能只删除标记。
6. `串口调试指南_Windows.md`，检查端口、命令与上下文是否一致。
7. `新手配置指南.md`，保留必要内容，避免重复安装步骤。

在当时 `.user` 的 modify/delete 冲突中，Git 曾把远端版本留在工作树供检查。这不表示 Git 已决定保留，也不代表它自动变成未跟踪文件；先打开看内容。
```bash
git add -- "ozone/yuntai.jdebug.user"
```
解释：只有决定保留并检查保存完该文件，才用此命令把保留结果暂存并标记处理完成。
```bash
git rm -- "ozone/yuntai.jdebug.user"
```
解释：这是“决定删除”的替代操作，会删除工作树文件并暂存删除，不能接在保留命令后照做。先备份有用内容，不因为扩展名是 `.user` 就直接删。

每个普通内容冲突都按“打开全部冲突位置 → 判断 → 编辑 → 删除冲突标记 → 保存 → 对该路径 add”处理。**add 只是认可当前内容，不会真正解决逻辑冲突。**
暂存后再次编辑也要重新 add。文件名含中文或空格时加英文引号；不要用一次全仓库暂存代替逐文件确认。

### 6.3 看自动合并内容，不只看七个冲突
```bash
git diff --cached --stat
```
解释：查看暂存的整体增删规模。合并会自动暂存无冲突文件，所以大量新增、删除可能正常，但仍必须审核。
```bash
git diff --cached --name-status
```
解释：列出暂存文件的新增、修改、删除等状态。特别核对历史中出现过的 `MDK-ARM/` 删除，以及 `.omo/`、`.pyc` 内容，不能仅因“自动合并”就认可，也不要趁机做无关清理。

```bash
git diff --cached --check
```
解释：再次检查最终暂存快照。冲突标记残留必须回编辑器处理后重新 add；空白提示逐项判断，不要把可能有意的 Markdown 换行当成代码冲突。

```bash
grep -nE '^(<<<<<<<|=======|>>>>>>>)' -- "Core/Src/my_main.cpp" "Mac迁移配置指南.md" "ozone/yuntai.jdebug" "ozone/yuntai.jdebug.user" "scripts/uart_read_win.py" "串口调试指南_Windows.md" "新手配置指南.md"
```
解释：在明确列出的工作区文件里辅助查找标记；按实际路径调整列表，已决定删除的文件应移出列表。文档中的示例或合法分隔线也可能匹配，要逐条看上下文；无匹配通常退出码 1，路径不存在则是检查错误，不算通过。

### 6.4 Gimbal 编译验证、完成或撤销

以下只针对 Gimbal 的历史 Debug 构建流程，不是自瞄仓库的构建命令。先确认终端在 Gimbal 根目录、冲突内容已整理完。
```bash
cmake --preset Debug
```
解释：使用项目的 Debug 预设配置构建。若预设不存在、工具链缺失或配置失败，停下检查实际工程说明，不继续假装编译成功。
```bash
cmake --build build/Debug
```
解释：配置成功后编译 Debug 构建目录。失败时找输出中第一个有意义的 error，例如语法错误或头文件缺失，而不是只看末尾汇总；修复、保存、重新暂存并再次验证。

编译通过只说明这套配置能构建，不证明控制逻辑、机械动作和硬件安全。本文流程**不烧录、不上电测试、不驱动电机**。

```bash
git status
```
解释：确认无 unmerged 项，并检查全部暂存内容。若提示冲突已修复但仍在 merging，下一步需要提交完成合并；无 U 不是代码正确的证明。
```bash
git commit -m "合并远端更新并解决冲突"
```
解释：只在确有待完成 merge 且审查验证完成时使用，记录合并结果。随后按仓库分支推送；自动成功的 merge 可能早已创建提交，不必再补一个空提交。
```bash
git merge --abort
```
解释：这是放弃未完成 merge 的替代路线，不是提交后的下一步。先把辛苦整理的内容另存仓库外；干净起步时通常能回到合并前，保留合并前已有本地提交。起步时已有未提交修改则未必能完整恢复，失败就停，不能接着硬重置。

如果 status 不处于 merging，尤其只是 stash apply 冲突，不要用这条撤销。先备份现状、辨认当前操作，再决定恢复办法。

## 七、差异、引号与 vi 编辑器

- 普通 diff 的 `+` 表示新内容，`-` 表示旧内容被移除，不是正确与错误。修改一句话常表现为一删一增。
- 冲突时 combined diff 可能有多列 `+`、`-`，是在与多个版本比较，不是“加号那边应该保留”；回编辑器读完整文件判断。
- `---`、`+++` 是文件标题，`@@` 或 `@@@` 是差异位置，不能当正文删改。
- `\ No newline at end of file` 只是某版本末尾没有换行，不等于内容丢失；需要时在编辑器补换行并重新暂存。
- 中文路径显示成反斜杠数字常是转义显示，不是文件名坏了；第三、六节的临时 quotePath 方式可让路径更直观。
- 日志或差异停在分页器里，按 `q` 退出查看，不会撤销操作。等待网络太久，可按 Ctrl+C 取消当前等待，再检查状态。

终端出现 `dquote>`，通常是双引号没闭合，命令还在等待输入。按 Ctrl+C 放弃这次输入，重新输入整条命令，确保英文引号成对，不要继续乱补文本。
提交时进入 vi/vim：普通模式用于操作，按 `i` 才进入插入模式；默认合并说明通常已经够用，不必修改。
保存退出：按 Esc，输入 `:wq`，按 Enter。`w` 保存、`q` 退出，Git 随后才能继续提交。
`:q!` 只是不保存本次编辑，不是可靠的 Git 取消方式：原有默认说明可能仍被 Git 使用。确要让 Vim 报失败退出可用 Esc 后 `:cq` 再 Enter，然后查看 status。
想避免编辑器，普通提交使用带说明的 `-m`，合并使用 `--no-edit` 或 `-m`；这些选项不免除检查内容的责任。

## 八、HTTPS、Clash 与令牌认证

### 8.1 先分清网络和权限

历史环境中 Clash 的 mixed 端口为 **7897**，现在是否仍监听要检查。浏览器能打开 GitHub，不代表终端 Git 使用了同一个代理。
```bash
nc -vz 127.0.0.1 7897
```
解释：检查本机代理端口是否有服务监听。连接拒绝先检查 Clash 是否启动、端口是否改变；成功也不代表代理能访问 GitHub。
```bash
curl -I --max-time 20 -x http://127.0.0.1:7897 https://github.com
```
解释：通过该 HTTP 代理测试 GitHub 网页响应，最多等待 20 秒。它不是 Git 仓库端点或写权限测试；只有代理 CONNECT 成功也不能证明最终 HTTPS 请求成功。
```bash
git -c http.proxy=http://127.0.0.1:7897 ls-remote origin
```
解释：临时经代理读取当前 HTTPS 远端的引用，更接近 Git 访问测试，但不下载合并，也不证明具有写权限。公开仓库可能无需认证即可读。

```bash
git -c http.proxy=http://127.0.0.1:7897 fetch origin
```
解释：只给这一次 Git 命令设置 HTTP 代理并下载更新，不改永久配置。仍要保护本地修改、检查差异并合并，fetch 不是直接覆盖本地文件。

```bash
git -c http.proxy=http://127.0.0.1:7897 push origin master
```
解释：确认该 HTTPS 仓库的 master 已审查、提交并包含所需远端更新后，临时经代理上传。不能为解决网络问题跳过合并，也不要把这条当 SSH 代理命令。

等待过久先 Ctrl+C，再排查代理。若中断的是 push，远端可能已经收到了提交；恢复网络后重新 fetch、比较历史，不要凭中断画面猜结果。

### 8.2 GitHub 账号与 PAT

HTTPS 提示 Username 时填 GitHub 账号，例如 `zy-2006-08`；Password 要填 Personal Access Token，不是网页登录密码。粘贴时终端通常不显示字符或星号，这是正常保护。
创建入口：[Fine-grained personal access tokens](https://github.com/settings/personal-access-tokens)，选择 Generate new token。
1. Resource owner 选择拥有目标仓库的账号或组织，不能选错所有者。
2. Repository access 选择需要的仓库；只允许自瞄的令牌不能自然获得云台、底盘写权限。
3. Repository permissions 中搜索 **Contents**，把它设为 **Read and write**，不是只读。
4. “Public repositories (read-only)”只提供公开仓库读取，不满足推送；组织审批、分支规则仍可能另外限制写入。
5. 设置有界有效期，记录到期提醒。即便界面允许 No expiration，也不保证永久有效，令牌仍可能被撤销或受策略影响。
6. 复制后存入可信密码管理器。不要写进仓库、远端 URL、命令参数、截图、聊天或日志；泄露后立即在 GitHub 撤销并重新生成。

`Invalid username or token` 优先查账号、粘贴是否完整、令牌过期或撤销、缓存是否还在用旧凭据。
`403` 优先查仓库选择、owner、Contents 写权限、组织审批与分支保护，不要把它一律解释成网络问题；权限不足时停止重试推送。

```bash
git -c credential.helper= -c http.proxy=http://127.0.0.1:7897 push origin master
```
解释：可选的一次性尝试，清空本次命令的 credential helper 列表，绕过该类缓存，再按提示输入凭据，不删除钥匙串也不修改永久配置。其他认证来源仍可能存在；只有提交范围已确认才运行，切勿把令牌直接补进命令。

## 九、SSH 超时与 443 端口

终端仓库历史远端是 SSH。HTTP 的代理选项不会把 SSH 的 22 端口流量自动送进 Clash；`port 22: Operation timed out` 应按 SSH 线路排查。

```bash
ssh -o ConnectTimeout=15 -T -p 443 git@ssh.github.com
```
解释：尝试 GitHub 的 SSH 443 入口，连接超时设为 15 秒。首次出现主机密钥询问，先对照官方指纹再输入 yes，不要盲目信任。

官方核验：[GitHub SSH key fingerprints](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/githubs-ssh-key-fingerprints)。按实际显示的算法逐字核对，不能拿仓库令牌代替主机指纹。
出现 `Hi 用户名! You've successfully authenticated, but GitHub does not provide shell access.` 表示认证成功，不提供 shell 是预期行为，这种测试也可能退出码为 1。
测试失败不一定是超时：`Permission denied (publickey)` 是密钥认证问题；主机密钥变化警告要先核实，不能直接删记录绕过；认证成功也不证明有目标仓库权限。

```bash
GIT_SSH_COMMAND="ssh -p 443 -o HostName=ssh.github.com" git fetch origin
```
解释：只给这次 SSH Git 下载指定 GitHub 443 入口，不修改远端 URL 或永久配置。必须在终端仓库执行；它不适用于那三个 HTTPS 远端，也不会完成合并。

历史上已经配置过 `~/.ssh/config`，包括保留 `Include /Users/zy/.colima/ssh_config`，以及 `Host github.com` 下的 `HostName ssh.github.com`、`Port 443`、`User git`。
这不是本次核验结果。需要时用编辑器只读查看该文件及匹配规则，不要整文件覆盖，不要重复追加 Host 块，避免破坏 Colima 或已有连接配置。
若该配置现在仍正确生效，普通 fetch 就会按配置走 443；它影响匹配 github.com 的 SSH 连接，不影响 HTTPS，也不保证以后网络永久可用。

```bash
git fetch origin
```
解释：在终端仓库使用现有 SSH 配置下载。成功后按 main 比较并合并；如仍报 22 超时，检查当前远端、配置匹配及环境覆盖，不要反复追加同一配置。

## 十、查克隆时间

```bash
git reflog show --date=iso HEAD
```
解释：查看本地 HEAD 的操作记录，寻找仍保留的 `clone: from ...` 条目及时间。reflog 可能过期、清理或来自迁移，找不到不证明没有克隆过；按 q 退出分页器。

```bash
stat -f '%SB' .git
```
解释：macOS 下查看 `.git` 的创建时间，作为近似线索。复制、迁移、恢复备份会影响它；worktree 的 `.git` 还可能只是文件，不能保证这是克隆日期。

提交时间代表作者或提交动作的时间，不是你下载仓库的时间。证据不足时结论写“无法确认”，不要用最早提交日期硬猜。

## 十一、日常配方与错误决策树

### 配方 A：三个 master 仓库只下载更新

进入上表对应根目录 → 保存并备份重要内容 → status 和 remote 核对 → fetch 成功 → 比较 HEAD 与 origin/master → 干净且未分叉时 ff-only 合入 → 再看 status。
有未提交修改插入第五节 stash 流程；双方分叉走第六节；只下载不需要为了流程完整而提交或推送。

### 配方 B：terminal 的 main 更新与上传

进入 `/Users/zy/RMGF/terminal` → status 核对 main → SSH fetch → 比较 HEAD 与 origin/main → 保护本地修改 → 快进或经审核的 merge → 必要时恢复 stash → 暂存检查并提交 → 推送 main。
所有比较和合并目标都用 `origin/main`，推送分支用 `main`，不要照抄 master。SSH 成功不会自动把未保存的文件上传。

### 配方 C：只想保存自己刚写的东西

保存编辑器 → status → 对目标文件 add → 阅读整个 cached diff 并 check → commit → 查看待上传提交 → 确认远端更新后按分支 push。
如果只是本机留版本，做到 commit 即可；想让 GitHub 留副本才 push。普通提交不应混入无关暂存文件；合并提交则需要审核并包含整个合并结果。

### 错误决策树：对号入座，处理完再继续

| 看到什么 | 下一步 | 明确停止条件 |
| --- | --- | --- |
| `not a git repository` | pwd，回到上表仓库根目录 | 目录不确定，不初始化、不提交 |
| 分支不是预期 main/master | 查看 status 和工程记录 | 不知道当前分支用途，不合并或推送 |
| 代理端口拒绝连接 | 查 Clash 运行状态与 mixed 端口 | 本机代理未监听，不重复认证 |
| HTTPS 超时 | 第八节分层测试网页与 Git 端点 | fetch 未成功，不把旧跟踪信息当最新 |
| invalid token / 403 | 分别核对认证与授权 | 不泄露令牌，不靠强推解决权限 |
| SSH 22 超时 | 第九节测试 443 并核对现有配置 | 主机指纹不符，不接受连接 |
| `non-fast-forward` / `fetch first` | fetch、保护改动、比较并合并后再 push | 不用 force 绕过别人的提交 |
| 本地或未跟踪文件将被覆盖 | 外部备份，按范围 stash 或人工安置 | 不明文件用途，不删除覆盖 |
| `Not possible to fast-forward` | 看双方独有提交，按第六节合并 | 不确定合并目标，不继续 |
| `CONFLICT` / unmerged | 列全 U，逐文件编辑保存再 add | 任何冲突未处理，不提交推送 |
| stash apply 冲突 | 保留 stash，整理已恢复内容 | 不重复 apply，不误用 merge abort |
| 编译失败 | 找第一个有意义错误，修复并复验 | 不宣称验证通过，不烧录 |
| `dquote>` / 卡在 vi | 按第七节退出等待或保存说明 | 未返回提示符，不连续粘更多命令 |

## 十二、最后检查清单

- [ ] 我知道当前目录、仓库、分支和 origin 地址，没把 main 与 master 混用。
- [ ] 重要文件已保存，必要的仓库外备份还在；没有把 stash 当永久异地备份。
- [ ] fetch 确实成功，双方提交已比较；clean 或空输出没有被误当成同步证明。
- [ ] 所有冲突都真正编辑处理过，没有只靠 add 消掉 U；修改后重新暂存。
- [ ] 检查的是全部暂存内容，特别是自动合并的大量删除、新增和可能的隐私文件。
- [ ] 需要的构建已成功；构建不等于硬件安全，没有顺手烧录或驱动设备。
- [ ] push 的是正确分支全部待上传提交，没用强推；报错或中断后重新核验了远端。
- [ ] 最后再看 status，保留无关改动与备份，不为“看起来干净”做额外删除。

记忆顺序：**先认仓库，再保存与备份；先下载看差异，再合并；先读内容，再暂存提交；最后确认范围才上传。**
