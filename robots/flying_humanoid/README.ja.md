# Flying Humanoid

[English](README.md) | 日本語

胴体に2つのジンバル付きロータを持つ二足ロボットです。飛行は gimbalrotor の制御器で行います。歩行は、軌道最適化
([crocoddyl](https://github.com/loco-3d/crocoddyl))で計画した軌道を再生して行い、体重の一部をロータで支えます。

## 環境構築

ROS だけが入った環境から始めます(Ubuntu 22.04/24.04 の ROS-O で、レポジトリルートの `configure.sh` を実行済みで、
`/opt/ros/one` がある状態)。以下のコマンドはすべてこの環境を前提にしています。

歩行には、jsk_aerial_robot の通常のパッケージに加えて次が必要です(飛行だけなら不要です)。

| パッケージ | 入手先 | |
|---|---|---|
| crocoddyl | レポジトリルートの `aerial_robot_*.rosinstall` | thrust-rate モデルを追加した fork(sugikazu75/crocoddyl の `PR/thrust_rate/exact_gradient`) |
| pinocchio (v4) | apt | |
| blasfeo, hpipm, hpipm-cpp | 別のワークスペース `~/ros/ocp_ws` | HPIPM-SQP ソルバー用([ソルバー](#ソルバー)を参照) |

### 1. apt パッケージ

```bash
sudo apt install -y python3-vcstool python3-catkin-tools python3-ipython coinor-libipopt-dev \
  ros-${ROS_DISTRO}-pinocchio ros-${ROS_DISTRO}-eigenpy
```

### 2. HPIPM 用のワークスペース(`~/ros/ocp_ws`)

これらの fork の `catkin` ブランチには、catkin でビルドできるように `package.xml` を追加してあります。

```bash
mkdir -p ~/ros/ocp_ws/src
cd ~/ros/ocp_ws/src
git clone -b catkin https://github.com/sugikazu75/blasfeo.git --recursive    # 各CPUのアーキテクチャに最適化された線形代数ライブラリ
git clone -b catkin https://github.com/sugikazu75/hpipm.git --recursive      # blasfeoをつかった内点法による最適制御ソルバ
git clone -b catkin https://github.com/sugikazu75/hpipm-cpp.git --recursive  # hpipmのC++ラッパー
source /opt/ros/${ROS_DISTRO}/setup.bash
cd ~/ros/ocp_ws
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build
```

### 3. ロボットのワークスペース(`~/ros/flying_humanoid_ws`)

最初の `catkin config` の前に `~/ros/ocp_ws` を source して、このワークスペースが ocp_ws を extend するようにします。
こうすると crocoddyl が hpipm-cpp を見つけて、HPIPM-SQP ソルバーもビルドされます。

- Release でビルドします。最適化なしでビルドすると、計画が大幅に遅くなります。
- `BUILD_WITH_MULTITHREADS` を付けると、計画器の `num_threads` を crocoddyl が使えるようになります。
- `BUILD_WITH_ODYN=OFF`: crocoddyl は ODYN が見つかれば使いますが、ここでは不要です。

```bash
mkdir -p ~/ros/flying_humanoid_ws/src
cd ~/ros/flying_humanoid_ws/src
git clone -b develop/flying_humanoid https://github.com/sugikazu75/jsk_aerial_robot.git
source ~/ros/ocp_ws/devel/setup.bash
cd ~/ros/flying_humanoid_ws
vcs import src < src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall --recursive   # crocoddyl もここで入る
rosdep install -y -r --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DBUILD_WITH_MULTITHREADS=ON -DBUILD_BENCHMARK=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTING=OFF -DBUILD_WITH_ODYN=OFF
catkin build
```

`~/ros/ocp_ws` を使わない場合は、手順2を飛ばし、手順3では代わりに `/opt/ros/${ROS_DISTRO}/setup.bash` を source します。
crocoddyl は HPIPM-SQP なしでビルドされるので、`config/WalkPlanner.yaml` で `solver_type: 0` と
`touchdown_constraint: false` にしてください。

## 実行方法

どの端末でも、最初に次を実行します。

```bash
source ~/ros/flying_humanoid_ws/devel/setup.bash
```

### 飛行

```bash
roslaunch flying_humanoid bringup.launch real_machine:=False simulation:=True headless:=False
rosrun aerial_robot_base keyboard_command.py   # 別の端末で。r でアーム、t で離陸、l で着陸
```

### 歩行

```bash
roslaunch flying_humanoid bringup.launch real_machine:=False simulation:=True headless:=False walk:=True
python3 $(rospack find flying_humanoid)/scripts/walk_interface.py   # 別の端末で。`robot` が使える IPython シェルが開く
```

```python
robot.start()
robot.takeoff()     # 歩行中もロータは回したまま(ナビゲータは takeoff 状態)
robot.plan(4)       # 現在の状態から4歩を計画し、制御器に読み込まれるまで待つ
robot.walk()        # 計画した軌道を再生する
robot.plan(2)       # 次の歩行も、歩く直前に計画し直す
robot.walk()
robot.walk(2)       # robot.plan(2) と robot.walk() をまとめて行う
```

`scripts/walk_interface.py` の `FlyingHumanoidInterface` は、自作のスクリプトからも使えます。

実機では、引数を `real_machine:=True simulation:=False`(デフォルト値)にします。

### 仕組み

```
walk_interface.py ──walk/plan_request (Int32: 歩数)──> walk_planner
                                                          uav/baselink/odom と joint_states から計画し、
                                                          /tmp/flying_humanoid_walk/walk_<N>steps_<時刻>.csv に保存
歩行制御器         <──walk/trajectory (String: CSV)───────┘
(aerial_robot_base)  歩行中でなければ読み込む ──walk/trajectory_loaded──> walk_interface.py
                  <──walk/start (Bool)──────────── walk_interface.py
                     再生する: joints_ctrl(脚)、gimbals_ctrl(ジンバル)、
                     four_axes/command(推力)、desire_coordinate(姿勢)
```

- `walk_planner`(`src/walk_planner*.cpp`、`config/WalkPlanner.yaml`)は、`bringup.launch walk:=True` で起動するノードです。
  - 計画した足の軌道(`lfoot_traj`、`rfoot_traj`)と、足と重心の参照(`lfoot_ref_traj`、`rfoot_ref_traj`、`com_ref_traj`)を、
    `world` フレームの Path として publish します。`config/rviz_config` で表示されます。
- 歩行制御器(`src/walk_controller.cpp`、`config/WalkControl.yaml`)は、gimbalrotor の制御器の代わりに
  `aerial_robot_base` が読み込む `aerial_robot_control` のプラグインです。
  - `walk/start` が来るまでは、軌道の最初のサンプルを保持します。
  - 推力は、軌道の値をフィードフォワードとし、ベースリンクの姿勢に対する PD フィードバックを加えて送ります。
  - 軌道が1つもない状態でアーム(`start`)されると、立つだけの軌道(0歩)を計画器に要求します。0歩の要求には最適化を
    行わず、現在の状態と QP で求めた静止推力の1サンプルだけが返ります。これで、歩行を計画する前でも離陸後に推力が送られます。

## 歩行計画器

歩行は、計画を要求した時点で計測した状態を基準に計画します。

- 歩く方向はロボットの向き(yaw)です。
- 各足の高さは計測値のままです。モーションキャプチャのオフセットで 0 にならないこともあるためです。
- そのため、計画は歩く直前に行ってください。
- `use_measured_state: false` にすると、代わりに原点に置いた `initial_configuration` から計画します。計画器だけを
  試すときに便利です。

計画の流れは次のとおりです。

1. 立つ。
2. 体重を左足に移す。
3. 右足から交互に踏み出す(各足は `step_length` ずつ進む)。
4. 最後に踏み出した足の横に、もう一方の足をそろえる。
5. 立つ。

推力は状態に含め、制御入力はその変化率にしています(thrust-rate モード)。そのため、計画される推力はなめらかに変化します。

`config/WalkPlanner.yaml` の主なパラメータ:

| パラメータ | |
|---|---|
| `step_length`, `step_height` | [m] |
| `time_step`, `step_knots`, `support_knots` | [s]、遊脚期と両脚支持期のノード数 |
| `lateral_shift_scale` | 重心を支持脚側へ寄せる量(0: 中央のまま) |
| `solver_type` | 0: FDDP、1: box-FDDP、3: HPIPM-SQP |
| `touchdown_constraint` | 遊脚を目標位置に速度0でぴったり着地させる(等式制約) |
| `touchdown_velocity_weight` | 着地時の速度0をコストとしても課す |
| `terminal_posture_constraint` | 終端の関節角度を `initial_configuration` の値にする(終端ノードの等式制約) |
| `terminal_thrust_weight` | 終端の推力を開始時の静止推力に戻す(終端ノードのコスト、下記参照) |
| `*_weight` | コストの重み |

`terminal_thrust_weight` がないと、終端の推力が左右で偏ったまま終わります(開始時 4.8 N / 4.8 N に対して、例えば
2.2 N / 3.9 N)。両脚支持中は、推力をどう配分しても接地力で釣り合ってしまうためです。

推力に等式制約をかけることはできません。crocoddyl の fork では、制約に渡されるのが q と v だけだからです。推力が
見えるのは、推力付きの状態で作ったコストモデルだけです。ただし、このコストモデルには多体の状態が必要なコスト
(重心、摩擦錐、角運動量)を入れられません。そのため終端ノードのコストは、状態に関するものだけになっています。

### ソルバー

各ソルバーが扱える等式制約:

| `solver_type` | 終端ノードの等式制約 | 途中のノードの等式制約 |
|---|---|---|
| 0: FDDP | 扱える | 扱えない |
| 1: box-FDDP | 扱えない | 扱えない |
| 3: HPIPM-SQP | 扱える | 扱える |

- FDDP と box-FDDP は、終端以外のノードの等式制約を満たす方向には動きません。それでも違反量は収束判定に含めるので、
  `touchdown_constraint: true` では収束しません。
- 終端に等式制約があると、box-FDDP は1イテレーション目で止まり、初期解をそのまま返します。
- 関節トルクを上下限内に収めるのは box-FDDP だけです。ただ、これまで計画したトルクはどれも上下限よりかなり小さい
  値でした(上限 1.5 Nm に対して 0.7 Nm 程度)。

まとめると、デフォルトの制約のまま HPIPM-SQP を使うか、`touchdown_constraint: false` にして FDDP を使ってください。
2〜4歩の歩行は、20〜30イテレーション程度、約1秒で収束します。

### 診断

`diagnosis: true`(`walk_planner` のプライベートパラメータ)にすると、軌道の CSV と同じ場所に次のファイルを保存します。

- `*_solver.csv`: イテレーションごとのソルバーの値(コスト、収束判定値、正則化、ステップ長、ダイナミクスのギャップ
  `ffeas`、等式制約の違反量 `hfeas`、予測改善量と実際の改善量など)
- `*_nodes.csv`: ノードごとの各コスト項の値(重み込み)、各等式制約の違反量、ダイナミクスのギャップ、制御入力の大きさ

また、いくつかのノードで、解析的な微分と数値微分の誤差を表示します。
