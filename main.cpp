#include <Novice.h>
#include "Class/Math/MyMath.h"

const char kWindowTitle[] = "LE2A_14_マツモトユウタ_MT4";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	Vector3 from0{ 1.0f,0.7f,0.5f };
	Vector3 to0 = -from0;
	Vector3 from1 { -0.6f,0.9f,0.2f };
	Vector3 to1{ 0.4f,0.7f,-0.5f };

	from1 = from1.Normalize();
	to1 = to1.Normalize();

	Matrix4x4 rotateMatrix0 = Matrix4x4::DirectionToDirection({ 1.0f,0.0f,0.0f }, { -1.0f,0.0f,0.0f });
	Matrix4x4 rotateMatrix1 = Matrix4x4::DirectionToDirection(from0, to0);
	Matrix4x4 rotateMatrix2 = Matrix4x4::DirectionToDirection(from1, to1);

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

		MatrixScreenPrintf(0, 0, rotateMatrix0, "rotateMatrix0");
		MatrixScreenPrintf(0, 122, rotateMatrix1, "rotateMatrix1");
		MatrixScreenPrintf(0, 244, rotateMatrix2, "rotateMatrix2");

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
