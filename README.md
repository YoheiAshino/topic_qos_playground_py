# topic_qos_playground_cpp

## 概要 / Overview
（日本語）
- ROS2のTopic通信において、QoS設定の違いがメッセージ配送に与える影響を、
  実験を通して確認するC++サンプルです。

(English)
- A C++ example to explore how different QoS settings affect ROS2 topic communication
  through practical experiments.

---

## 目的 / Why
（日本語）
- QoS（reliability, depth など）の基本的な挙動を整理し、
  制御系・周期処理における設計判断を説明できるようにするため。

(English)
- To understand the behavior of ROS2 QoS settings and be able to explain
  design decisions for control loops and periodic systems.

---

## 環境 / Environment
- ROS 2: Humble
- OS: Ubuntu
- Language: C++

---

## 実行方法 / How to run

### Build
```bash
cd ~/ros2_ws
colcon build --packages-select topic_qos_playground_cpp --symlink-install
source install/setup.bash
```

## Run（QoS切替）
Reliable:
```bash
ros2 launch topic_qos_playground_cpp demo.launch.py reliable:=true depth:=10
```
Best effort:
```bash
ros2 launch topic_qos_playground_cpp demo.launch.py reliable:=false depth:=10
```

## インタフェース / Interfaces
Nodes
- qos_talker
Publishes std_msgs/msg/String
- qos_listener
Subscribes std_msgs/msg/String

Topics
- Publish: /chatter (std_msgs/msg/String)
- Subscribe: /chatter (std_msgs/msg/String)

Parameters
- reliable (bool, default: true)
	- true: reliable
	- false: best_effort
- depth (int, default: 10)
 QoS KeepLast のキュー深さ

## 実験 / Experiment
### 目的
Subscriber が Publisher の送信周期に追いつけない状況を意図的に作り、
QoS 設定によって メッセージ欠落（drop） と
遅延蓄積（delay） がどのように現れるかを観測する。

## 実験条件（例）
- Publisher 周期: 10ms
- Subscriber 側処理遅延: 約20ms
- QoS キュー深さ: depth = 1
起動例：
```bash
ros2 launch topic_qos_playground_cpp demo.launch.py reliable:=false depth:=1
```
```bash
ros2 launch topic_qos_playground_cpp demo.launch.py reliable:=true depth:=1
```

## 観測方法
Subscriber 側では、受信したメッセージに含まれる連番を用いて、
連続性が崩れた場合に警告ログを出力する。
```sql
WARN: DROP? expected last+1 but got n
```
このログは、
「前回受信した番号 last の次（last+1）が来るはずだが、
実際には n が先に観測された」
ことを意味する。
※ この時点では 欠落（drop）か遅延（delay）かは確定していない。

## 観測結果と解釈
### best_effort QoS
- 受信番号が大きく飛ぶことが多く、
DROP? expected X but got Y が頻繁に出力される
- 飛ばされた番号のメッセージは 後から到着することはなく、
実際に欠落していると判断できる
- ros2 topic hz の値も揺れやすく、受信が間引かれる傾向が見られる
→ best_effort では、追いつけない場合に古いメッセージが破棄される
（実際の drop が発生する）

### reliable QoS
- 同様の警告ログが一時的に出る場合があるが、
その後に過去の番号のメッセージが遅れて出力されることがある
- ログ出力がしばらく止まり、その後まとめて出力されるなど、
遅延が蓄積して解放される挙動が観測される
→ メッセージ自体は欠落しておらず、
Subscriber 側で遅延が蓄積して観測順が乱れている状態

## まとめ
- DROP? ログは 「連番の連続性が崩れた」ことを示す指標であり、
それ自体が即「欠落」を意味するわけではない
- best_effort では主に 欠落（drop） として現れ、
reliable では 遅延の蓄積（delay） として現れることが多い
- 制御系や周期処理では、
「欠落」と「遅延」のどちらが許容されるかを考慮して
QoS を選択する必要がある

## 設計メモ / Design Notes
- 状態推定や制御入力など「欠落が致命的な経路」では reliable が有効
- 一方で、周期制御では「欠落」よりも「遅延」の方が危険になる場合があり、
用途やシステム特性に応じた QoS 設計が重要である

## Related / 関連
- Python版: topic_qos_playground_py（同一実験を Python で再現）

## Next
- Python版との比較（実装量・可読性・実験のしやすさ）を追記
- timestamp を用いた遅延の定量評価
- 実験条件（publish周期 / subscriber処理時間）の外部パラメータ化
