# topic_qos_playground_py

## 概要 / Overview
（日本語）
- ROS2のTopic通信において、QoS設定の違いがメッセージ配送に与える影響を、
  Pythonでの実験を通して確認するサンプルです。
- C++版と同一条件の実験を行い、挙動の再現性と実装の違いを比較できます。

(English)
- A Python example to explore how different ROS2 QoS settings affect topic communication
  through practical experiments.
- This repository mirrors the C++ version under the same conditions for comparison.

---

## 目的 / Why
（日本語）
- ROS2 QoS（reliability, depth など）の挙動を、
  Pythonで素早く検証・可視化できる形で整理するため。
- C++版と同一実験を行い、言語による実装・検証体験の違いを理解する。

(English)
- To quickly experiment with and visualize ROS2 QoS behavior using Python.
- To compare implementation style and observability between C++ and Python.

---

## 環境 / Environment
- ROS 2: Humble
- OS: Ubuntu
- Language: Python (rclpy)

---

## 実行方法 / How to run

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select topic_qos_playground_py --symlink-install
source install/setup.bash
```

## Run（QoS切替）
### Reliable:
```bash
ros2 launch topic_qos_playground_py demo.launch.py reliable:=true
```
### Best effort:
```bash
ros2 launch topic_qos_playground_py demo.launch.py reliable:=false
```
## インタフェース / Interfaces
### Nodes
- `talker_py` 
  Publishes `std_msgs/msg/String`
- `listener_py` 
  Subscribes `std_msgs/msg/String`

### Topics
- Publish: `/chatter` (`std_msgs/msg/String`)
- Subscribe: `/chatter` (`std_msgs/msg/String`)

### Parameters
- `reliable` (bool, default: `true`) 
  - `true`: reliable 
  - `false`: best_effort
- `depth` (int, default: `10`) 
  QoS KeepLast のキュー深さ
- `pub_period_ms` (int, default: `10`) 
  Publisher の送信周期（実験用）
- `sub_sleep_ms` (int, default: `20`) 
  Subscriber 側の人工的な処理遅延（実験用）

---

## 実験 / Experiment

### 目的
Subscriber が Publisher の送信周期に追いつけない状況を意図的に作り、 
QoS 設定によって **メッセージ欠落（drop）** と 
**遅延蓄積（delay）** がどのように現れるかを観測する。

---

### 実験条件（例）
- Publisher 周期: 10ms 
- Subscriber 側処理遅延: 約20ms 
- QoS キュー深さ: `depth = 1`

起動例：

```bash
ros2 launch topic_qos_playground_py demo.launch.py reliable:=false depth:=1 pub_period_ms:=10 sub_sleep_ms:=20
```

### 観測方法
Subscriber 側では、受信メッセージに含まれる連番を用いて、 
**連続性が崩れた場合に警告ログを出力**する。
```sql
WARN: DROP? expected last+1 but got n
```
このログは、 
「前回受信した番号 `last` の次（`last+1`）が来るはずだが、 
実際には `n` が先に観測された」 
ことを意味する。

※ この時点では **欠落（drop）か遅延（delay）かは確定していない**。

---

### 観測結果と解釈

#### best_effort QoS
- 受信番号が大きく飛ぶことが多く、 
  `DROP? expected X but got Y` が頻繁に出力される
- 飛ばされた番号のメッセージは **後から到着することはなく**、 
  実際に欠落していると判断できる
- `ros2 topic hz` の値も揺れやすく、 
  受信が間引かれる傾向が見られる

→ **best_effort では、Subscriber が追いつけない場合に 
古いメッセージが破棄される（実際の drop が発生する）**

---

#### reliable QoS
- 一時的に警告ログが出ることがあるが、 
  その後に過去の番号のメッセージが遅れて出力される場合がある
- ログ出力がしばらく止まり、 
  その後まとめて出力されるなど、 
  Subscriber 側に遅延が蓄積する挙動が確認できる

→ **メッセージ自体は欠落しておらず、 
遅延として観測されている状態**

---

### まとめ
- `DROP?` ログは **「連番の連続性が崩れた」ことを示す指標**であり、 
  それ自体が即「欠落」を意味するわけではない
- best_effort では主に **欠落（drop）** として現れ、 
  reliable では **遅延の蓄積（delay）** として現れることが多い
- 制御系や周期処理では、 
  「欠落」と「遅延」のどちらが許容されるかを考慮して 
  QoS を選択する必要がある

---

## C++版との比較 / Comparison with C++

| 観点 | C++ | Python |
|---|---|---|
| 実行性能 | 高い | 低め |
| 実験の書きやすさ | 普通 | 高い |
| ログ・可視化 | やや手間 | 容易 |
| 検証・試作 | △ | ◎ |

---

## Related / 関連
- C++版: `topic_qos_playground_cpp`

---

## Next
- timestamp を用いた遅延の定量評価 
- Python / C++ 両方での実験結果の統合整理 
- 他QoS設定（history, durability）の追加検証


