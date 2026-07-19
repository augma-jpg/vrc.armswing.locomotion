"""
armswing.py — VRChat 腕振りロコモーション

腕を振って移動する VRChat 用ロコモーションツール。両手コントローラのグリップを
握りながら腕を振ると、振り速度に応じた速さで「体の向き」に移動する。
頭（HMD）ではなく体の向きに進むため、周囲を見回しながら移動できる。

機能:
  - グリップゲート: 両手グリップを握っている間だけ移動（誤動作防止）
  - 振り速度に応じた歩走の速度マッピング
  - 体の向き基準の移動（両手の振り方とコントローラ向きのハイブリッド推定）
  - 後退モード: グリップ + 人差し指トリガーで後退
  - 腕振りジャンプ: 両手を HMD より高く素早く振り上げるとジャンプ
  - Ctrl+C で安全終了（全入力を 0 にリセット）

前提ファイル（同フォルダ）:
  armswing.vrmanifest / armswing_actions.json / bindings_oculus_touch.json

必要環境・使い方は README.md を参照。

注意（入力フォーカス）:
  IVRInput のアクションは、本アプリに入力フォーカスがある時だけ bActive=True になる。
  VRChat 実行中は VRChat がシーンアプリとしてフォーカスを供給するため正常動作する。
"""

import os
import time
import csv
import math
import cmath
import argparse
import datetime
import threading

import openvr
from pythonosc import udp_client
from pythonosc import dispatcher as osc_dispatcher
from pythonosc import osc_server

# ============================================================
# チューニング対象パラメータ
# ============================================================
POLL_HZ = 60           # メインループ周波数
ALPHA_ATTACK = 0.2     # EMA 係数（上昇時＝腕振り加速時）。大きいほど移動発動が機敏
ALPHA_RELEASE = 0.05   # EMA 係数（下降時）。小さいほど折り返しの谷を粘り、停止時のコーストが長い
V_ON = 0.3             # 歩行開始閾値 [m/s]
V_OFF = 0.15           # 歩行停止閾値 [m/s]
V_MIN = 0.3            # マッピング下限 [m/s]（V_ON と同値推奨）
V_MAX = 1.5            # マッピング上限 [m/s]
HICCUP_LIMIT = 0.5     # 外れ値除去の 1 フレーム移動上限 [m]
GRIP_THRESHOLD = 0.5   # アナロググリップを「握った」とみなす閾値（0..1）

DIR_ALPHA = 0.03          # 方向の円形EMA係数（時定数 ≈ 0.56 秒）。
                          # 大きくすると θ_body がスイング揺らぎに速く追従し
                          # 足元がふらつく感覚（横入力の微振動）が出るため、
                          # 実機 A/B 比較（0.05/0.04/0.03）で 0.03 に確定。
                          # 起動時引数 --dir-alpha で切り替え可能
HOR_SPEED_MIN = 0.15      # 方向サンプルとして採用する水平速さの下限 [m/s]
TRIGGER_THRESHOLD = 0.5   # 後退モードのトリガー閾値（0..1）
HORIZONTAL_SIGN = -1.0    # Horizontal = HORIZONTAL_SIGN · s · sin(δ)。
                          # 実機の受け入れテストで確定した符号
LOST_RESET_SEC = 0.5      # 両手 invalid がこの秒数続いたら v_raw=0（速度凍結の暴走防止）
RESCAN_INTERVAL_SEC = 1.0 # 手が invalid の間、この間隔でデバイス index を再スキャン

CAL_OFFSET_DEG = 0.0      # 方向キャリブレーション [度]。θ_body に常時加算する。
                          # 握り方由来の系統バイアス（±10度前後）が気になる場合に設定。
                          # 測り方: 目印に正対して直進 → --log の delta_deg 列の
                          # 中央値の符号反転値を設定
JUMP_VY_MIN = 1.2         # 腕振りジャンプ: 両手の上向き速度の下限 [m/s]
                          # （実測の実ジャンプ動作は vy≈2.5 m/s。余裕をみた値）
JUMP_COOLDOWN = 0.4       # ジャンプ再発火禁止時間 [s]。目的は /input/Jump の
                          # 誤連射（OSC 無駄送信）の抑制のみ。VRChat 側が着地まで
                          # 再ジャンプ不可を保証するためゲームは破綻しない。
                          # 設計初期値 1.0 は「ジャンプして物の上に乗る」ケース
                          # （実測で約0.5秒後に着地・再ジャンプ可能）に間に合わない
                          # ため 0.4 に短縮
JUMP_PULSE_SEC = 0.1      # /input/Jump=1 の保持時間 [s]（ボタン型入力のパルス幅）

# ============================================================
# 環境設定
# ============================================================
DURATION_SEC = None             # None なら Ctrl+C まで
CSV_BASENAME = "armswing_log"   # --log 指定時、実行ごとに日時付き別ファイルに保存

OSC_SEND_IP = "127.0.0.1"      # VRChat への入力送信先
OSC_SEND_PORT = 9000
OSC_LISTEN_IP = "127.0.0.1"    # VRChat output の受信
OSC_LISTEN_PORT = 9001

# ---- IVRInput マニフェスト関連 ----
HERE = os.path.dirname(os.path.abspath(__file__))
ACTION_MANIFEST_PATH = os.path.join(HERE, "armswing_actions.json")
APP_MANIFEST_PATH = os.path.join(HERE, "armswing.vrmanifest")
APP_KEY = "armswing.locomotion"
ACTION_SET = "/actions/armswing"
LEFT_GRIP_ACTION = "/actions/armswing/in/LeftGrip"
RIGHT_GRIP_ACTION = "/actions/armswing/in/RightGrip"
LEFT_TRIGGER_ACTION = "/actions/armswing/in/LeftTrigger"
RIGHT_TRIGGER_ACTION = "/actions/armswing/in/RightTrigger"
NO_RESTRICT = openvr.k_ulInvalidInputValueHandle

# ============================================================
# 円統計ヘルパ（角度の平均・差は必ず円統計で）
# ============================================================
def wrap_deg(angle):
    """任意の値の角度を -180～180度 に変換する。"""
    return (angle + 180.0) % 360.0 - 180.0


def circ_diff_deg(a, b):
    """円差 a - b [度]（[-180, 180) に折り返し）。±180 の巻き戻りで壊れない。"""
    return wrap_deg(a - b)


def circ_mean_deg(angles):
    """角度リストの円平均 [度]。単位複素数 e^(i·yaw) の平均の偏角。
    ほぼ正反対で相殺した場合は None。"""
    if not angles:
        return None
    z = sum(cmath.exp(1j * math.radians(a)) for a in angles)
    if abs(z) < 1e-9:
        return None
    return math.degrees(cmath.phase(z))


# ============================================================
# 体の向き推定（ハイブリッド方式）
# ============================================================
class BodyDirectionEstimator:
    """体の向き θ_body [度] を両手の振り方（軸）と両コントローラの向き(yaw)を
    ヒントにハイブリッド方式で推定するクラス。

    毎フレーム:
      1. コントローラ yaw の円平均 cym を計算
      2. 各手の水平速度 (vx, vz) について、水平速さ sp ≥ hor_speed_min のものだけ
         ang = atan2(vx, vz) を取り、cym との円差が ±90度超なら 180度足して符号解決。
         サンプル sp·e^(i·ang) を作る（速さで重み付け）
      3. 採用サンプルの平均を正規化し、円形EMAで複素アキュムレータ Z を更新:
         Z = dir_alpha·z + (1 − dir_alpha)·Z、θ_body = arg(Z)
         採用サンプルが無いフレームは更新しない（前回の θ_body を保持＝方向凍結）
      4. 初期値はゲートON瞬間の cym でシード（呼び出し側が seed() する）

    OpenVR 非依存の純粋ロジック。
    """

    def __init__(self, dir_alpha=DIR_ALPHA, hor_speed_min=HOR_SPEED_MIN):
        self.dir_alpha = dir_alpha
        self.hor_speed_min = hor_speed_min
        self.Z = None  # 身体の向きの角度θを算出する為の複素数。None = 未登録

    @property
    def theta(self):
        """現在の推定 θ_body [度]。未登録なら None。"""
        if self.Z is None or abs(self.Z) < 1e-9:
            return None
        return math.degrees(cmath.phase(self.Z))

    def seed(self, yaw_deg):
        """θ_body をゲートON瞬間の両コントローラ yaw 円平均で初期化。"""
        if yaw_deg is not None:
            self.Z = cmath.exp(1j * math.radians(yaw_deg))

    def update(self, hand_velocities, controller_yaws):
        """1フレーム分の更新。ゲートON中のみ呼ぶこと。

        hand_velocities: このフレームに新鮮に計算できた手の水平速度 [(vx, vz), ...]
                         （invalid・外れ値・初回フレームの手は含めない）
        controller_yaws: 有効なコントローラの yaw [度] のリスト（0〜2個）
        返り値: 更新後の θ_body [度]（未シードのままなら None）
        """
        cym = circ_mean_deg(controller_yaws)
        if self.Z is None:
            if cym is None:
                return None
            self.seed(cym)  # シード前に update が呼ばれた場合の保険
        if cym is not None:
            samples = []
            for vx, vz in hand_velocities:
                sp = math.hypot(vx, vz)
                if sp < self.hor_speed_min:
                    continue
                ang = math.degrees(math.atan2(vx, vz))
                # 符号解決: コントローラが向いている側を「前」とする（180度曖昧性の解消）
                if abs(circ_diff_deg(ang, cym)) > 90.0:
                    ang += 180.0
                samples.append(sp * cmath.exp(1j * math.radians(ang)))
            if samples:
                z = sum(samples) / len(samples)
                if abs(z) > 1e-9:
                    z /= abs(z)  # 速さ重みはサンプル平均の向きにのみ寄与させる
                    self.Z = self.dir_alpha * z + (1.0 - self.dir_alpha) * self.Z
                    if abs(self.Z) < 1e-6:
                        self.Z = z  # 数値ガード（ほぼ完全相殺で偏角が不定になるのを防ぐ）
        return self.theta


# ============================================================
# 腕振りジャンプ判定
# ============================================================
class JumpDetector:
    """腕振りジャンプのトリガー判定。OpenVR 非依存の純粋ロジック。

    毎フレーム update() を呼ぶ。トリガー条件:
      jump_condition = gate_on
                       and (left_y > hmd_y) and (right_y > hmd_y)
                       and (left_vy > vy_min) and (right_vy > vy_min)
    - 高さ・速度のいずれかが不明（None）のフレームは条件不成立として扱う
      （invalid・外れ値・初回フレームの手はジャンプ判定に使わない）
    - 発火は立ち上がりエッジ（条件 False→True）のみ。条件が成立し続けても
      1 ジェスチャーにつき 1 回しか発火しない
    - 発火後 cooldown 秒は再発火しない（/input/Jump の誤連射抑制が目的。
      クールダウン中の立ち上がりは捨てられ、次の False→True を待つ）
    - gate_on 必須: 手ぶらの両手上げ（伸び・バンザイ等）での誤爆防止
    """

    def __init__(self, vy_min=JUMP_VY_MIN, cooldown=JUMP_COOLDOWN):
        self.vy_min = vy_min
        self.cooldown = cooldown
        self.prev_condition = False
        self.last_fire_time = None

    def update(self, now, gate_on, hmd_y, left_y, right_y, left_vy, right_vy):
        """1フレーム分の判定。

        now:              現在時刻 [s]（単調増加なら基準は問わない）
        gate_on:          両手グリップゲート
        hmd_y/left_y/right_y: HMD・両手の高さ [m]。invalid なら None
        left_vy/right_vy: 両手の垂直速度 [m/s]。このフレームに新鮮に計算できて
                          いなければ None
        返り値: (condition, fired)
          condition: このフレームのトリガー条件（ログ用）
          fired:     このフレームで /input/Jump パルスを開始すべきなら True
        """
        values = (hmd_y, left_y, right_y, left_vy, right_vy)
        condition = bool(
            gate_on
            and all(v is not None for v in values)
            and left_y > hmd_y and right_y > hmd_y
            and left_vy > self.vy_min and right_vy > self.vy_min
        )
        fired = False
        if condition and not self.prev_condition:
            if (self.last_fire_time is None
                    or (now - self.last_fire_time) >= self.cooldown):
                fired = True
                self.last_fire_time = now
        self.prev_condition = condition
        return condition, fired


# ============================================================
# OSC 受信（VRChat output）— ログ用に取得
# ============================================================
osc_cache = {
    "VelocityX": None, "VelocityY": None, "VelocityZ": None,
    "AngularY": None, "Upright": None, "Grounded": None,
}


def make_osc_handler(param_name):
    def handler(address, *args):
        if args:
            osc_cache[param_name] = args[0]
    return handler


def start_osc_server():
    disp = osc_dispatcher.Dispatcher()
    for name in osc_cache.keys():
        disp.map(f"/avatar/parameters/{name}", make_osc_handler(name))
    server = osc_server.ThreadingOSCUDPServer(
        (OSC_LISTEN_IP, OSC_LISTEN_PORT), disp)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


# ============================================================
# OpenVR ヘルパ
# ============================================================
def find_device_indices(vr):
    hmd_index = left_index = right_index = None
    for i in range(openvr.k_unMaxTrackedDeviceCount):
        device_class = vr.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_HMD:
            hmd_index = i
        elif device_class == openvr.TrackedDeviceClass_Controller:
            role = vr.getControllerRoleForTrackedDeviceIndex(i)
            if role == openvr.TrackedControllerRole_LeftHand:
                left_index = i
            elif role == openvr.TrackedControllerRole_RightHand:
                right_index = i
    return hmd_index, left_index, right_index


def get_position(pose):
    m = pose.mDeviceToAbsoluteTracking
    return m[0][3], m[1][3], m[2][3]


def get_rotation_ypr(pose, flip_if_upside_down=False):
    """3x4行列の左3x3（回転）から yaw / pitch / roll（度）を取り出す。

    flip_if_upside_down=True（コントローラ用。案E「upベクトル補正」）:
      デバイスの up ベクトルのワールド y 成分は m[1][1]。これが負＝上下さかさまの間、
      オイラー分解は |roll|>90度 の表現に切り替わり、yaw は実際の向きから約180度
      反転して報告される（激しい腕振りで手が垂直を跨いだ瞬間がこれに当たる）。
      その間だけ yaw に 180度 を足して打ち消す。
      HMD には適用しない（トラッキング瞬断時に hmd_yaw 経由で進行方向が反転する
      新しい故障モードを作らないため）。
    """
    m = pose.mDeviceToAbsoluteTracking
    yaw = math.degrees(math.atan2(m[0][2], m[2][2]))
    if flip_if_upside_down and m[1][1] < 0.0:
        yaw = wrap_deg(yaw + 180.0)
    sp = max(-1.0, min(1.0, -m[1][2]))
    pitch = math.degrees(math.asin(sp))
    roll = math.degrees(math.atan2(m[1][0], m[1][1]))
    return yaw, pitch, roll


# ============================================================
# IVRInput セットアップ
# ============================================================
def register_application():
    """アプリマニフェストを登録し、実行プロセスをアプリキーに紐付ける。

    identifyApplication が UnknownApplication になる事象への堅牢化込み:
      - 登録状態（isApplicationInstalled）を診断表示
      - identify をリトライ（SteamVR 側のマニフェスト反映待ち）
      - 永続登録が反映されない場合は一時登録（temporary）にフォールバック
    """
    if not os.path.exists(APP_MANIFEST_PATH):
        raise FileNotFoundError(f"アプリマニフェスト不在: {APP_MANIFEST_PATH}")
    apps = openvr.VRApplications()
    try:
        apps.addApplicationManifest(APP_MANIFEST_PATH, False)
    except Exception as e:
        print(f"  addApplicationManifest 注記: {e}")
    print(f"  アプリ登録状態: installed={apps.isApplicationInstalled(APP_KEY)}")

    last_err = None
    for attempt in range(10):
        try:
            apps.identifyApplication(os.getpid(), APP_KEY)
            if attempt > 0:
                print(f"  identifyApplication: リトライ {attempt} 回で成功")
            return
        except openvr.error_code.ApplicationError as e:
            last_err = e
            # 数回失敗しても永続登録が見えない場合は一時登録を試す
            if attempt == 3 and not apps.isApplicationInstalled(APP_KEY):
                print("  永続登録が反映されないため一時登録(temporary)を試行します")
                try:
                    apps.addApplicationManifest(APP_MANIFEST_PATH, True)
                except Exception as e2:
                    print(f"  addApplicationManifest(temporary) 注記: {e2}")
            time.sleep(0.5)
    print("  identifyApplication が失敗し続けました。切り分け:")
    print(f"  - installed={apps.isApplicationInstalled(APP_KEY)} "
          f"(False ならマニフェスト登録自体が拒否されている)")
    print("  - SteamVR の再起動 / armswing.vrmanifest の binary_path_windows が")
    print("    実在ファイルを指しているかを確認してください")
    raise last_err


def setup_input(vr_input):
    """アクションマニフェスト登録 → ハンドル取得 → アクティブセット配列作成。"""
    if not os.path.exists(ACTION_MANIFEST_PATH):
        raise FileNotFoundError(f"アクションマニフェスト不在: {ACTION_MANIFEST_PATH}")
    vr_input.setActionManifestPath(ACTION_MANIFEST_PATH)
    set_handle = vr_input.getActionSetHandle(ACTION_SET)
    handles = {
        "grip_left": vr_input.getActionHandle(LEFT_GRIP_ACTION),
        "grip_right": vr_input.getActionHandle(RIGHT_GRIP_ACTION),
        "trig_left": vr_input.getActionHandle(LEFT_TRIGGER_ACTION),
        "trig_right": vr_input.getActionHandle(RIGHT_TRIGGER_ACTION),
    }

    active = (openvr.VRActiveActionSet_t * 1)()
    active[0].ulActionSet = set_handle
    active[0].ulRestrictedToDevice = NO_RESTRICT
    active[0].nPriority = 0
    return handles, active


# ============================================================
# 速度計算（外れ値除去つき）
# ============================================================
def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def hand_velocity(idx, poses, prev_pos, prev_speed, dt):
    """1 手のフレーム間速度を返す。

    返り値: (speed, velocity, new_prev_pos)
      speed:    v_raw 用の速さ [m/s]。invalid・外れ値は前回速度を保持
      velocity: (vx, vy, vz) [m/s]。このフレームで新鮮に計算できた時のみ。
                invalid・外れ値・初回フレームは None（方向サンプル・ジャンプ判定に
                使わない）
      new_prev_pos: 次フレームの基準にする位置
    """
    # トラッキング無効 → 前回速度を保持し、基準位置も維持（復帰時の飛びを防ぐ）
    if idx is None or not poses[idx].bPoseIsValid:
        return prev_speed, None, prev_pos

    pos = get_position(poses[idx])
    if prev_pos is None:
        return 0.0, None, pos

    dx = pos[0] - prev_pos[0]
    dy = pos[1] - prev_pos[1]
    dz = pos[2] - prev_pos[2]
    dist = math.sqrt(dx * dx + dy * dy + dz * dz)

    # 外れ値（トラッキング異常）: 前回速度を保持し、基準位置は維持
    if dist > HICCUP_LIMIT:
        return prev_speed, None, prev_pos

    if dt <= 0:
        return 0.0, None, pos
    return dist / dt, (dx / dt, dy / dt, dz / dt), pos


# ============================================================
# メイン
# ============================================================
def parse_args():
    parser = argparse.ArgumentParser(
        description="VRChat 腕振りロコモーション")
    parser.add_argument(
        "--log", action="store_true",
        help="日時付き CSV ログを記録する（開発・チューニング用。デフォルト OFF）")
    parser.add_argument(
        "--dir-alpha", type=float, default=DIR_ALPHA, metavar="A",
        help=f"方向の円形EMA係数（デフォルト {DIR_ALPHA}。"
             f"実機 A/B 比較用。例: --dir-alpha 0.03）")
    return parser.parse_args()


def main():
    args = parse_args()

    vr = openvr.init(openvr.VRApplication_Background)
    print("OpenVR に接続しました")

    register_application()
    vr_input = openvr.VRInput()
    handles, active = setup_input(vr_input)
    print(f"IVRInput 準備完了（app={APP_KEY}）")

    osc_srv = start_osc_server()
    client = udp_client.SimpleUDPClient(OSC_SEND_IP, OSC_SEND_PORT)
    print(f"OSC 送信先 {OSC_SEND_IP}:{OSC_SEND_PORT} / "
          f"受信 {OSC_LISTEN_IP}:{OSC_LISTEN_PORT}")
    print(f"DIR_ALPHA={args.dir_alpha} / CAL_OFFSET_DEG={CAL_OFFSET_DEG} / "
          f"ジャンプ: vy>{JUMP_VY_MIN}m/s, cooldown={JUMP_COOLDOWN}s / "
          f"CSVログ: {'ON' if args.log else 'OFF（--log で有効化）'}")

    hmd_index, left_index, right_index = find_device_indices(vr)
    print(f"HMD={hmd_index}, Left={left_index}, Right={right_index}")
    if left_index is None or right_index is None:
        print("警告: 認識されていないコントローラがあります（1秒ごとに再スキャンします）")

    # CSV ログは --log 指定時のみ（開発・チューニング用）。
    # ファイル名に dir_alpha を含める（A/B 走行ログの識別用）
    csv_path = None
    csv_file = None
    writer = None
    if args.log:
        csv_path = (f"{CSV_BASENAME}_dirAlpha{args.dir_alpha:g}_"
                    f"{datetime.datetime.now():%Y%m%d_%H%M%S}.csv")
        csv_file = open(csv_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(csv_file)
        writer.writerow([
            "timestamp", "elapsed_sec",
            "hmd_x", "hmd_y", "hmd_z",
            "left_x", "left_y", "left_z",
            "right_x", "right_y", "right_z",
            "hmd_valid", "left_valid", "right_valid",
            "hmd_yaw", "hmd_pitch", "hmd_roll",
            "left_yaw", "left_pitch", "left_roll",
            "right_yaw", "right_pitch", "right_roll",
            "vel_x", "vel_y", "vel_z",
            "angular_y", "upright", "grounded",
            "grip_left", "grip_right", "gate_on",
            "v_raw", "v_smooth", "walking", "sent_vertical",
            "trig_left", "trig_right", "reverse_active",
            "theta_body", "delta_deg", "sent_horizontal",
            "left_vy", "right_vy", "jump_condition", "jump_fired",
        ])

    interval = 1.0 / POLL_HZ
    start = time.time()
    prev_time = start

    # 速度・平滑化・歩行状態
    prev_pos = {"left": None, "right": None}
    prev_speed = {"left": 0.0, "right": 0.0}
    v_smooth = 0.0
    walking = False

    # 方向推定の状態
    estimator = BodyDirectionEstimator(dir_alpha=args.dir_alpha)
    prev_gate = False
    last_hmd_yaw = None            # HMD が一瞬 invalid でも直近の yaw で合成を続ける
    both_lost_frames = 0           # 両手 invalid の連続フレーム数（凍結対策）
    lost_reset_frames = int(LOST_RESET_SEC * POLL_HZ)
    last_rescan = start            # index 再スキャンのタイマ

    # ジャンプ状態
    jump_detector = JumpDetector()
    jump_pulse_until = None        # /input/Jump=1 を 0 に戻す時刻。None = パルス外

    print(f"動作開始（{POLL_HZ}Hz）。Ctrl+C で停止します。\n")

    try:
        while True:
            now = time.time()
            elapsed = now - start
            dt = now - prev_time
            prev_time = now
            if DURATION_SEC is not None and elapsed >= DURATION_SEC:
                break

            # ---- ポーズ取得 ----
            poses = vr.getDeviceToAbsoluteTrackingPose(
                openvr.TrackingUniverseStanding, 0,
                openvr.k_unMaxTrackedDeviceCount)

            def pose_data(idx, flip_if_upside_down=False):
                if idx is None or not poses[idx].bPoseIsValid:
                    return ("", "", "", "", "", "", False)
                x, y, z = get_position(poses[idx])
                yaw, pitch, roll = get_rotation_ypr(poses[idx], flip_if_upside_down)
                return (x, y, z, yaw, pitch, roll, True)

            # コントローラのみ案E補正を適用（ログの left_yaw / right_yaw も補正後の値。
            # 以後のオフライン再生が実機と同じ入力を再現できるようにするため）
            hx, hy, hz, hyaw, hpitch, hroll, hmd_valid = pose_data(hmd_index)
            lx, ly, lz, lyaw, lpitch, lroll, left_valid = pose_data(
                left_index, flip_if_upside_down=True)
            rx, ry, rz, ryaw, rpitch, rroll, right_valid = pose_data(
                right_index, flip_if_upside_down=True)

            # ---- デバイス index 再スキャン ----
            # 左右いずれかの手が invalid の間、1秒ごとに再スキャンし、変化があれば差し替える
            if (not left_valid or not right_valid) and \
                    (now - last_rescan) >= RESCAN_INTERVAL_SEC:
                last_rescan = now
                new_h, new_l, new_r = find_device_indices(vr)
                if (new_h, new_l, new_r) != (hmd_index, left_index, right_index):
                    print(f"\n[再スキャン] index 変更: "
                          f"HMD {hmd_index}->{new_h}, "
                          f"L {left_index}->{new_l}, R {right_index}->{new_r}")
                    # index が変わった手は位置基準をリセット（旧デバイスとの差分で
                    # 速度を計算しない。外れ値ガードの恒久ホールドも防ぐ）
                    if new_l != left_index:
                        prev_pos["left"] = None
                    if new_r != right_index:
                        prev_pos["right"] = None
                    hmd_index, left_index, right_index = new_h, new_l, new_r

            # ---- ゲート判定（両手グリップ）＋ 後退判定（トリガー）----
            vr_input.updateActionState(active)
            la = vr_input.getAnalogActionData(handles["grip_left"], NO_RESTRICT)
            ra = vr_input.getAnalogActionData(handles["grip_right"], NO_RESTRICT)
            lt = vr_input.getAnalogActionData(handles["trig_left"], NO_RESTRICT)
            rt = vr_input.getAnalogActionData(handles["trig_right"], NO_RESTRICT)
            grip_left = la.x
            grip_right = ra.x
            trig_left = lt.x
            trig_right = rt.x
            gate_on = (bool(la.bActive) and bool(ra.bActive)
                       and grip_left >= GRIP_THRESHOLD
                       and grip_right >= GRIP_THRESHOLD)
            reverse_active = gate_on and (
                (bool(lt.bActive) and trig_left >= TRIGGER_THRESHOLD)
                or (bool(rt.bActive) and trig_right >= TRIGGER_THRESHOLD))

            # ---- 速度計算（外れ値除去つき）----
            sp_l, vel_l, prev_pos["left"] = hand_velocity(
                left_index, poses, prev_pos["left"], prev_speed["left"], dt)
            sp_r, vel_r, prev_pos["right"] = hand_velocity(
                right_index, poses, prev_pos["right"], prev_speed["right"], dt)
            prev_speed["left"] = sp_l
            prev_speed["right"] = sp_r
            v_raw = (sp_l + sp_r) / 2.0   # 両手平均

            # ---- 両手ロスト時の速度凍結対策 ----
            # 「前回速度保持」は短時間のヒカップ用。両手とも invalid が 0.5 秒続いたら
            # v_raw=0 として扱い、最後の速度のまま走り続ける暴走を防ぐ
            if not left_valid and not right_valid:
                both_lost_frames += 1
            else:
                both_lost_frames = 0
            if both_lost_frames >= lost_reset_frames:
                v_raw = 0.0

            # ---- EMA 平滑化（Attack-Release 非対称）----
            alpha = ALPHA_ATTACK if v_raw > v_smooth else ALPHA_RELEASE
            v_smooth = alpha * v_raw + (1.0 - alpha) * v_smooth

            # ---- ヒステリシス（歩行状態）----
            if gate_on:
                if not walking and v_smooth > V_ON:
                    walking = True
                elif walking and v_smooth < V_OFF:
                    walking = False
            else:
                # ゲート解除は「止める」明確な意思表示。惰性を持ち越さず即停止
                walking = False
                v_smooth = 0.0

            # ---- 方向推定（ハイブリッド方式）----
            yaws = []
            if left_valid:
                yaws.append(lyaw)
            if right_valid:
                yaws.append(ryaw)
            if gate_on and not prev_gate:
                # ゲートON瞬間のコントローラ yaw 円平均でシード
                estimator.seed(circ_mean_deg(yaws))
            if gate_on:
                # θ_body の更新はゲートON中のみ（OFF中は動かないので凍結で無害）
                hor_vels = []
                if vel_l is not None:
                    hor_vels.append((vel_l[0], vel_l[2]))
                if vel_r is not None:
                    hor_vels.append((vel_r[0], vel_r[2]))
                estimator.update(hor_vels, yaws)
            prev_gate = gate_on
            theta_body = estimator.theta

            # ---- 腕振りジャンプ判定 ----
            left_vy = vel_l[1] if vel_l is not None else None
            right_vy = vel_r[1] if vel_r is not None else None
            jump_condition, jump_fired = jump_detector.update(
                now, gate_on,
                hy if hmd_valid else None,
                ly if left_valid else None,
                ry if right_valid else None,
                left_vy, right_vy)
            if jump_fired:
                client.send_message("/input/Jump", 1)
                jump_pulse_until = now + JUMP_PULSE_SEC
            elif jump_pulse_until is not None and now >= jump_pulse_until:
                client.send_message("/input/Jump", 0)
                jump_pulse_until = None

            # ---- 入力合成 ----
            if hmd_valid:
                last_hmd_yaw = hyaw
            if gate_on and walking:
                s = clamp((v_smooth - V_MIN) / (V_MAX - V_MIN), 0.0, 1.0)
            else:
                s = 0.0
            if theta_body is not None and last_hmd_yaw is not None:
                # CAL_OFFSET_DEG: 握り方由来の系統バイアスの手動補正（デフォルト 0）
                delta = circ_diff_deg(theta_body + CAL_OFFSET_DEG, last_hmd_yaw)
            else:
                delta = 0.0   # フォールバック: 頭（HMD）基準の前進
            if reverse_active:
                delta = wrap_deg(delta + 180.0)   # 後退モード: ベクトルごと反転
            delta_rad = math.radians(delta)
            sent_vertical = s * math.cos(delta_rad)
            sent_horizontal = HORIZONTAL_SIGN * s * math.sin(delta_rad)

            client.send_message("/input/Vertical", sent_vertical)
            client.send_message("/input/Horizontal", sent_horizontal)

            # ---- ログ（--log 指定時のみ）----
            if writer is not None:
                ts = datetime.datetime.now().isoformat(timespec="milliseconds")
                writer.writerow([
                    ts, f"{elapsed:.3f}",
                    hx, hy, hz, lx, ly, lz, rx, ry, rz,
                    hmd_valid, left_valid, right_valid,
                    hyaw, hpitch, hroll,
                    lyaw, lpitch, lroll,
                    ryaw, rpitch, rroll,
                    osc_cache["VelocityX"], osc_cache["VelocityY"],
                    osc_cache["VelocityZ"],
                    osc_cache["AngularY"], osc_cache["Upright"],
                    osc_cache["Grounded"],
                    f"{grip_left:.3f}", f"{grip_right:.3f}", gate_on,
                    f"{v_raw:.4f}", f"{v_smooth:.4f}", walking,
                    f"{sent_vertical:.4f}",
                    f"{trig_left:.3f}", f"{trig_right:.3f}", reverse_active,
                    f"{theta_body:.2f}" if theta_body is not None else "",
                    f"{delta:.2f}", f"{sent_horizontal:.4f}",
                    f"{left_vy:.3f}" if left_vy is not None else "",
                    f"{right_vy:.3f}" if right_vy is not None else "",
                    jump_condition, jump_fired,
                ])

            # ---- コンソール 1 行更新 ----
            hands_str = (f"L{'o' if left_valid else 'x'}"
                         f"R{'o' if right_valid else 'x'}")
            gate_str = "ON " if gate_on else "off"
            rev_str = "REV" if reverse_active else "---"
            jump_str = "JMP" if jump_pulse_until is not None else "---"
            print(f"\r t={elapsed:6.1f}s {hands_str} gate={gate_str} rev={rev_str} "
                  f"jmp={jump_str} v={v_smooth:4.2f} d={delta:+6.1f} "
                  f"V={sent_vertical:+5.2f} H={sent_horizontal:+5.2f}  ",
                  end="", flush=True)

            sleep_time = interval - (time.time() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n停止要求を受けました")

    finally:
        # 安全: 何があっても入力を 0 に戻す（Vertical / Horizontal / Jump の3つ）
        try:
            client.send_message("/input/Vertical", 0.0)
            client.send_message("/input/Horizontal", 0.0)
            client.send_message("/input/Jump", 0)
        except Exception:
            pass    # OSC 送信失敗でも以下の終了処理は続行する
        if csv_file is not None:
            csv_file.close()
        osc_srv.shutdown()
        openvr.shutdown()
        if csv_path is not None:
            print(f"\n入力を 0 リセットし切断しました。ログ: {csv_path}")
        else:
            print("\n入力を 0 リセットし切断しました")


if __name__ == "__main__":
    main()
