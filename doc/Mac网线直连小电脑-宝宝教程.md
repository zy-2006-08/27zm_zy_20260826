# Mac 用网线直连小电脑：宝宝教程

用一根网线把 Mac 和车上的小电脑直接连起来，再用 SSH 登录小电脑。不需要路由器，也不需要联网。插着线时来回只要 0.4 毫秒左右，是最快、最稳的连法。

第 1 到第 8 节是 x86 小电脑的做法，地址和设置在 zy（Ubuntu 26.04）+ MacBook 上实测能用（2026 年 11 月）。第 9 节讲 Jetson 载板有什么不同，那一节没在真 Jetson 上测过，标了 **【未验证】** 的地方第一次做时顺手确认，然后把结论改回这份文档。

连好以后是这样：

```
Mac ── USB-C 转网口 ── 网线 ── 小电脑的网口
192.168.10.3                    192.168.10.2
上网照旧走 Wi-Fi                 上网照旧走 Wi-Fi
```

---

## 0. 准备

| 东西 | 说明 |
| --- | --- |
| 网线 | 普通网线就行，不用买"交叉线"，现在的网口会自己适配 |
| USB-C 转网口转接头 | MacBook 没有网口。买写着"免驱"或"支持 macOS"的，插上就能用 |
| 小电脑的显示器和键盘 | 第一次设置要在小电脑上敲命令。已经能用别的办法（比如 Wi-Fi）SSH 上去的话，在 SSH 里做也行 |

## 1. 插线

转接头插在 Mac 上，网线一头插转接头，另一头插小电脑的网口。两边网口的小灯应该会亮或者闪。

## 2. 先弄懂要填的地址

网线直连时中间没有路由器，没人给两台电脑分配地址，所以要自己给每台定一个固定地址：

| | 填什么 |
| --- | --- |
| 小电脑的地址 | `192.168.10.2` |
| Mac 的地址 | `192.168.10.3` |
| 子网掩码 | `255.255.255.0`（命令里写成 `/24`） |
| 路由器（网关） | **不填** |

- 两个地址前三段一样（都是 `192.168.10`），两台才能直接找到对方。
- 路由器不填的原因：填了，电脑会以为"上网也走这根线"，可线那头的小电脑不给你上网，结果就断网了。不填，这根线只用来让两台互相连，上网照旧走 Wi-Fi。
- 别和这些号段撞：`172.20.10.x`（iPhone 热点）、`192.168.55.x`（Jetson 的 USB 网络）、`192.168.12.x`（终端连选手端的那根线）、你当前 Wi-Fi 的号段（dzkd-syl 是 `192.168.200.x`）。

## 3. 小电脑这边（只做一次）

> zy 这台已经配好了（配置名叫 `netplan-enp1s0`），可以直接跳到第 4 节。

在小电脑上按 `Ctrl+Alt+T` 打开终端。

**第一步：确认 SSH 服务开着**

```bash
sudo apt install -y openssh-server
ss -tln | grep ':22 '
```

第一行要联网；已经装过的话，它会提示"已经是最新版"。第二行有输出，而且里面有 `LISTEN` 和 `:22`，就对了。

**第二步：找到网口的名字**

```bash
ip -brief link
```

以 `en` 或 `eth` 开头的是网口，比如 `enp1s0`、`eno1`、`eth0`。插着网线的那个显示 `UP`，没插的显示 `DOWN`。有两个网口时，看哪个是 `UP`。zy 上插线的是 `enp1s0`。

**第三步：给网口设固定地址**

把命令里的 `enp1s0` 换成第二步看到的名字，整行复制：

```bash
sudo nmcli connection add type ethernet ifname enp1s0 con-name mac-direct connection.autoconnect-priority 10 ipv4.method manual ipv4.addresses 192.168.10.2/24 ipv4.never-default yes ipv6.method ignore
sudo nmcli connection up mac-direct
```

| 这段 | 意思 |
| --- | --- |
| `con-name mac-direct` | 给这套设置起个名字，以后改它、删它都用这个名字 |
| `ipv4.addresses 192.168.10.2/24` | 小电脑的地址和子网掩码 |
| `ipv4.never-default yes` | 上网别走这根线，也就是上面说的"路由器不填" |
| `connection.autoconnect-priority 10` | 比系统自带的"有线连接 1"优先，以后开机插着线就自动用这套 |
| `ipv6.method ignore` | 这根线不用 IPv6，省得乱 |

检查一下：

```bash
ip -brief addr show enp1s0
```

看到 `192.168.10.2/24` 就对了。

**第四步：看一眼防火墙**

```bash
sudo ufw status
```

显示"不活动"（inactive）就什么都不用做。显示"激活"（active）的话，运行：

```bash
sudo ufw allow ssh
sudo ufw allow 60000:61000/udp
```

第二行是给 mosh 用的，不用 mosh 可以不加。

**第五步：记下用户名**

```bash
whoami
```

zy 上是 `seven`。

## 4. Mac 这边（只做一次）

1. 打开"系统设置 → 网络"，点有线网卡。名字一般带"以太网"、"LAN"或"USB"，比如 `USB 10/100/1000 LAN`。
2. 点"详细信息…"，左边选"TCP/IP"。
3. "配置 IPv4"选"手动"。IP 地址填 `192.168.10.3`，子网掩码填 `255.255.255.0`，**路由器空着**，点"好"。

## 5. 测试：Mac 能不能找到小电脑

打开 Mac 的"终端"，运行：

```bash
ping -c 3 192.168.10.2
```

通了是这样，`time` 一般在 1 毫秒以下：

```
64 bytes from 192.168.10.2: icmp_seq=0 ttl=64 time=0.412 ms
...
3 packets transmitted, 3 packets received, 0.0% packet loss
```

显示 `Request timeout` 的话，去第 8 节查。

## 6. 登录小电脑

```bash
ssh seven@192.168.10.2
```

`seven` 换成第 3 节第五步看到的用户名。

1. 第一次连会问 `Are you sure you want to continue connecting (yes/no/[fingerprint])?`，输入 `yes` 回车。
2. 输入小电脑的登录密码。输入时屏幕上不显示字，这是正常的。
3. 开头变成 `seven@zy:~$` 这样，就连上了。现在敲的命令都在小电脑上运行。输入 `exit` 回到 Mac。

## 7. 连上以后（可选）

**不用每次输密码**

在 Mac 的终端里运行（看清那一行结尾是 `%`，不是 `$`，`$` 说明你还在小电脑上）：

```bash
ssh-keygen -t ed25519
```

一路回车。如果问 `Overwrite (y/n)?`，输入 `n`。这说明你的 Mac 上已经有密钥了，别覆盖它。然后运行：

```bash
ssh seven@192.168.10.2 'mkdir -p ~/.ssh && chmod 700 ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys' < ~/.ssh/id_ed25519.pub
```

输入一次密码。以后 ssh、mosh、VS Code 连小电脑都不用再输密码。

**用 VS Code**

装 `Remote - SSH` 插件 → 点左下角 `><` → "连接到主机…" → 输入 `seven@192.168.10.2` → 系统选 `Linux`。

**用 mosh（合盖、断网不掉线）**

小电脑上运行 `sudo apt install -y mosh tmux`，Mac 上运行 `brew install mosh`。然后在 Mac 上：

```bash
LC_ALL=zh_CN.UTF-8 mosh seven@192.168.10.2 -- tmux new -A -s main
```

开头的 `LC_ALL=...` 要填小电脑上有的语言包。在小电脑上运行 `locale -a`：有 `zh_CN.utf8` 就填 `zh_CN.UTF-8`（zy 就是这种），只有 `en_US.utf8` 就填 `en_US.UTF-8`。填错了，mosh 会报 UTF-8 或 locale 相关的错误。

## 8. 出问题先查这里

| 现象 | 先查 |
| --- | --- |
| `ping` 显示 `Request timeout` | ① 两头网口的灯亮不亮 ② 小电脑上运行 `ip -brief addr`，网口是不是 `UP`，有没有 `192.168.10.2` ③ Mac 的地址是不是 `192.168.10.3`、子网掩码是不是 `255.255.255.0` ④ 第 3 节命令里的网口名字填对没有 |
| `nmcli connection up` 报 `no carrier`，或者网口一直是 `DOWN` | 线没插紧；换根线；换小电脑上另一个网口；检查转接头插好没有 |
| `Connection refused` | 小电脑的 SSH 服务没开，重做第 3 节第一步 |
| `Permission denied` | 用户名或密码不对 |
| `REMOTE HOST IDENTIFICATION HAS CHANGED` | 同一个地址换了一台小电脑，或者小电脑重装过系统。在 Mac 上运行 `ssh-keygen -R 192.168.10.2`，然后再连 |
| `ping` 通了，`ssh` 一直卡着 | 小电脑的防火墙挡了，看第 3 节第四步 |
| Mac 插上网线后上不了网 | Mac 网卡设置里"路由器"那栏填了东西，把它清空 |
| 小电脑插上网线后上不了网 | 命令里漏了 `ipv4.never-default yes`。补上：`sudo nmcli connection modify mac-direct ipv4.never-default yes`，再运行 `sudo nmcli connection up mac-direct` |
| 小电脑这个网口插到路由器上，拿不到地址、上不了网 | 这个口被设成了固定地址，专门给 Mac 直连用。插小电脑上另一个网口；只有一个网口的话，先删掉这套设置：`sudo nmcli connection delete mac-direct`，以后要直连 Mac 时，再按第 3 节第三步加回来 |
| 设错了想重来 | `sudo nmcli connection delete mac-direct`，然后重做第 3 节第三步 |

## 9. 如果小电脑是 Jetson 载板

> 这一节没在真 Jetson 上测过，是按 NVIDIA 的文档和论坛整理的。

整体步骤一样：Jetson 跑的也是 Ubuntu（NVIDIA 叫它 JetPack），也用 NetworkManager，第 1 到第 8 节的命令照样能用。不一样的地方有这几处：

| 地方 | x86 小电脑（比如 zy） | Jetson |
| --- | --- | --- |
| 系统 | 普通 Ubuntu | JetPack：JetPack 5 是 Ubuntu 20.04，JetPack 6 是 22.04，JetPack 7 是 24.04。CPU 是 ARM，`apt` 用法一样 |
| 网口名字 | `enp1s0` 这类 | Orin Nano / NX 开发套件一般是 `enP8p1s0`，AGX Orin 是 `eno1`，老一些的 Jetson 常见 `eth0`，第三方载板各不一样。**一定要先用 `ip -brief link` 看，再填进第 3 节的命令** |
| 用户名 | 装系统时设的 | 刷机后第一次开机时设的；厂商预装的系统看说明书。NVIDIA 官方镜像没有默认账号 |
| mosh 的语言包 | zy 只有中文，填 `zh_CN.UTF-8` | 【未验证】一般有 `en_US.utf8`，填 `LC_ALL=en_US.UTF-8`，以 `locale -a` 为准 |
| 网口完全没反应 | 少见 | 【未验证】第三方载板的网口驱动在厂商给的系统里。`ip -brief link` 里一个 `en` 或 `eth` 开头的都没有，说明系统缺驱动，要刷厂商的镜像 |

**Jetson 多一种连法：只用一根 USB 数据线**

Jetson 开发套件开机后，用 USB 数据线连 Mac 和 Jetson 的 USB-C 口，Jetson 会把自己变成一块 USB 网卡，地址固定是 `192.168.55.1`，Mac 会自动拿到 `192.168.55.100`。两边都不用设置，直接运行：

```bash
ssh 用户名@192.168.55.1
```

但在 Mac 上这条路不太稳：

- 有人反映 JetPack 7.2.1 配 MacBook 连不上，降到 JetPack 6.2.1 就好了。NVIDIA 的回复是，这种连法建议电脑用 Ubuntu，并且用 Type-A 转 Type-C 的线（[NVIDIA 论坛](https://forums.developer.nvidia.com/t/jetpack-7-2-1-ssh-connection-issue-with-macos/383426)）。
- 线必须能传数据，只能充电的线不行。
- 第三方载板不一定把这个口接成这种模式，看载板说明书。
- 这根 USB 线不给 Jetson 上网。

所以用 Mac 的话，网线最靠谱，USB 线可以留着当备用。

参考资料：[Jetson AI Lab 入门教程](https://www.jetson-ai-lab.com/tutorials/getting-started-with-jetson/)、[NVIDIA 关于 Jetson 网口名字的文档](https://docs.nvidia.com/jetson/jps/platform-services/networking.html)。
