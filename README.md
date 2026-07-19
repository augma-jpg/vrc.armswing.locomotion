# Armswing Locomotion for VRChat

腕を振って移動する VRChat 用ロコモーションツールです。

> **English**: An arm-swinging locomotion tool for VRChat on SteamVR, aimed at
> reducing VR motion sickness. Swing your arms while holding both grips to move
> in the direction your **body** (not your head) is facing — so you can look
> around or even look back while running. Requires Windows, SteamVR, Python 3
> and OSC enabled in VRChat. Documentation below is in Japanese.

## 概要

スティック移動による VR 酔いの軽減を目的に、両手のグリップを握って腕を振ると
その振り速度に応じて移動するツールです。頭（HMD）ではなく**体の向き**に進むため、
「周囲を見回しながら移動する」「振り返りながら走る」ことができます。

<!-- TODO: デモ GIF をここに追加 -->

## 特徴

- **グリップゲート**: 両手グリップを握っている間だけ移動。誤動作を防止
- **振り速度に応じた歩走**: ゆっくり振れば歩き、速く振れば走る
- **体の向き基準の移動**: 両手の振り方とコントローラの向きから体の向きを推定
- **後退モード**: グリップ + 人差し指トリガーで後退
- **腕振りジャンプ**: 両手を頭より高く素早く振り上げるとジャンプ
- **追加トラッカー不要**: Quest 3 のコントローラのみで動作

## 必要環境

- Meta Quest 3（他の SteamVR 対応 HMD は未確認）
- Windows PC
- Meta Horizon Link（旧 Quest Link）
- SteamVR
- Python 3.x

起動順序は「Meta Horizon Link → SteamVR → VRChat → 本スクリプト」です。

## インストール

```
git clone https://github.com/augma-jpg/vrc.armswing.locomotion.git
cd vrc.armswing.locomotion
python -m pip install -r requirements.txt
```

## 使い方

1. VRChat の設定で **OSC を Enabled** にする
2. VRChat を起動した状態で実行:

   ```
   python armswing.py
   ```

3. 操作:
   - **前進**: 両手のグリップを握ったまま腕を振る（振り速度で歩走が変わる）
   - **後退**: グリップに加えて人差し指トリガーを引きながら腕を振る
   - **ジャンプ**: グリップを握ったまま両手を頭より高く素早く振り上げる
   - **終了**: コンソールで `Ctrl+C`（移動入力をすべて 0 にリセットして安全に終了）

## 初回起動について

初回起動時に SteamVR へアプリとして自動登録されます。**手動のバインド設定は
不要**です。SteamVR 設定 > コントローラ > バインド管理に「ARMSWING LOCOMOTION」
が現れますが、これは正常な動作です（カスタムバインドもここから可能です）。

## オプション

| オプション | 説明 |
|---|---|
| `--log` | 日時付き CSV ログを記録する（開発・チューニング用。デフォルト OFF） |
| `--dir-alpha A` | 体の向き推定の追従係数（デフォルト 0.03。大きいほど機敏だがふらつきやすい） |

## トラブルシューティング

- **動かない** → VRChat が起動しているか確認（SteamVR のシーンアプリが必須）/
  VRChat の OSC が Enabled になっているか確認
- **グリップが効かない** → SteamVR 設定 > コントローラ > バインド管理に
  「ARMSWING LOCOMOTION」が登録されているか確認 / スクリプトを再起動
- **ジャンプしない** → ワールドがジャンプを許可しているか確認

## 既知の注意点

- ハンドジェスチャーがアバターの表情に紐づいている場合、グリップ操作で表情が
  変わります。気になる場合はジェスチャートグルを OFF にすることを推奨します
- 移動以外の腕動作（剣を振る、シャドーボクシング等）との併用は非対応です
- 移動中の体の向き変更はスナップターンの使用を推奨します

## 免責

本ツールは自己責任でご使用ください。実際に身体を動かすため、周囲に十分な
スペースを確保し、安全に注意して使用してください。

## ライセンス

[MIT License](LICENSE)

<!-- TODO: 解説ブログ記事へのリンク（Zenn 公開後に追記） -->
