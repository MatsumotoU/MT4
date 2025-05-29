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

	Quaternion q1{};
	q1.q = { 2.0f,3.0f,4.0f,1.0f };

	Quaternion q2{};
	q2.q = { 1.0f,3.0f,5.0f,2.0f };

	Quaternion identity{};
	identity = Quaternion::IndentityQuaternion();

	Quaternion conj{};
	conj = Quaternion::ConjugationQuaternion(q1);

	Quaternion inv = Quaternion::Inverse(q1);

	Quaternion normal = Quaternion::Normalize(q1);

	Quaternion mul1 = Quaternion::Multiply(q1, q2);

	Quaternion mul2 = Quaternion::Multiply(q2, q1);

	float norm = Quaternion::Norm(q1);

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

		VectorScreenPrintf(0, 0, identity.q, "Identity");
		VectorScreenPrintf(0, 20, conj.q, "Conjugate");
		VectorScreenPrintf(0, 40, inv.q, "Inverse");
		VectorScreenPrintf(0, 60, normal.q, "Normalize");

		VectorScreenPrintf(0, 100, mul1.q, "Multiply(q1,q2)");
		VectorScreenPrintf(0, 120, mul2.q, "Multiply(q2,q1)");

		Novice::ScreenPrintf(0, 140, "%.2f :%s", norm, "Norm");

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
