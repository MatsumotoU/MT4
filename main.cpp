#include <Novice.h>
#include "Class/Math/MyMath.h"
#include "Class/Math/Quaternion/Quaternion.h"

const char kWindowTitle[] = "LE2A_14_マツモトユウタ_MT4";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Quaternion rotation0 = Quaternion::MakeRotateAxisAngleQuaternion({ 0.71f,0.71f,0.0f }, 0.3f);
	Quaternion rotation1 = Quaternion::MakeRotateAxisAngleQuaternion({ 0.71f,0.0f,0.71f }, 3.141592f);

	Quaternion interpolate0 = Quaternion::Slerp(rotation0, rotation1, 0.0f);
	Quaternion interpolate1 = Quaternion::Slerp(rotation0, rotation1, 0.3f);
	Quaternion interpolate2 = Quaternion::Slerp(rotation0, rotation1, 0.5f);
	Quaternion interpolate3 = Quaternion::Slerp(rotation0, rotation1, 0.7f);
	Quaternion interpolate4 = Quaternion::Slerp(rotation0, rotation1, 1.0f);

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		VectorScreenPrintf(0, 0, interpolate0.q, "Sleap(q0,q1,0.0f)");
		VectorScreenPrintf(0, 20, interpolate1.q, "Sleap(q0,q1,0.3f)");
		VectorScreenPrintf(0, 40, interpolate2.q, "Sleap(q0,q1,0.5f)");
		VectorScreenPrintf(0, 60, interpolate3.q, "Sleap(q0,q1,0.7f)");
		VectorScreenPrintf(0, 80, interpolate4.q, "Sleap(q0,q1,1.0f)");

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
