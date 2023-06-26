//
//	Dept. Software Convergence, Kyung Hee University
//	Prof. Daeho Lee, nize@khu.ac.kr
//
#include "KhuGleWin.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <functional>
#include <cmath>

#pragma warning(disable:4996)

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#ifdef _DEBUG
#ifndef DBG_NEW
#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
#define new DBG_NEW
#endif
#endif  // _DEBUG


// ENVIRONMENT VARIABLES
bool X2CHECKORBOARD = false;
bool X2UPSCALE = false;
bool X4BLINEAR = false;
bool X4PIXELSCALE = false;
bool X4CUBIC = false;
bool DRAWCLIFF = true;
bool EDGEVISUAL = false;
bool NORMALVISUAL = false;
bool ROTATE = true;

// CHECK RENDER INFO
int renderTriangle = 0;
int renderPixel = 0;

struct CKgTriangle {
	CKgVector3D v0, v1, v2;

	CKgTriangle() : v0(CKgVector3D()), v1(CKgVector3D()), v2(CKgVector3D()) {};
	CKgTriangle(CKgVector3D vv0, CKgVector3D vv1, CKgVector3D vv2)
		: v0(vv0), v1(vv1), v2(vv2) {};
};

class ObjLoader
{
public:
	std::vector<CKgVector3D> vertexs;
	std::vector<std::tuple<int, int, int>> faces;
	std::vector<std::tuple<int, int, int>> faceNorms;
	std::vector<std::tuple<int, int, int>> faceUvs;
	std::vector<CKgVector3D> normals;
	std::vector<CKgVector3D> uvs;
public:
	void LoadObjFromFile(std::string filepath, bool inverse = false) {
		int z = inverse ? -1 : 1;
		std::ifstream fin;
		fin.open(filepath.c_str());

		std::string line;
		while (std::getline(fin, line)) {
			if (line.length() != 0) {
				if (line[0] == 'v' && line[1] == ' ') {
					std::stringstream ss(line);
					std::string var, one, two, three;
					ss >> var >> one >> two >> three;
					vertexs.push_back(CKgVector3D(std::stold(one), std::stold(two), std::stold(three) * z));
				}
				else if (line[0] == 'f' && line[1] == ' ') {

					std::stringstream ss(line);
					std::string var, v0, v0n, v0t, v1, v1n, v1t, v2, v2n, v2t, temp;
					getline(ss, temp, ' '); // f

					getline(ss, temp, ' ');
					std::stringstream sss(temp);
					getline(sss, v0, '/');
					getline(sss, v0t, '/');
					getline(sss, v0n);

					getline(ss, temp, ' ');
					std::stringstream ssss(temp);
					getline(ssss, v1, '/');
					getline(ssss, v1t, '/');
					getline(ssss, v1n);

					getline(ss, temp);
					std::stringstream sssss(temp);
					getline(sssss, v2, '/');
					getline(sssss, v2t, '/');
					getline(sssss, v2n);

					faces.push_back(std::make_tuple(std::stoi(v0) - 1, std::stoi(v1) - 1, std::stoi(v2) - 1));
					faceNorms.push_back(std::make_tuple(std::stoi(v0n) - 1, std::stoi(v1n) - 1, std::stoi(v2n) - 1));
					faceUvs.push_back(std::make_tuple(std::stoi(v0t) - 1, std::stoi(v1t) - 1, std::stoi(v2t) - 1));
				}
				else if (line[0] == 'v' && line[1] == 'n' && line[2] == ' ') {
					std::stringstream ss(line);
					std::string var, one, two, three;
					ss >> var >> one >> two >> three;
					normals.push_back(CKgVector3D(std::stod(one), std::stod(two), std::stod(three) * z));
				}
				else if (line[0] == 'v' && line[1] == 't' && line[2] == ' ') {
					std::stringstream ss(line);
					std::string var, one, two;
					ss >> var >> one >> two;
					uvs.push_back(CKgVector3D(std::stod(one), std::stod(two), 0));
				}
			}
		}
		std::cout << "ModelInfo Vertex:" << vertexs.size() << " Faces:" << faces.size() << std::endl;
	}
};

class CKhuGle3DSprite : public CKhuGleSprite {
public:
	std::vector<CKgTriangle> SurfaceMesh;
	std::vector<CKgTriangle> Normal;
	std::vector<CKgTriangle> Uv;
	double** m_ProjectionMatrix;
	CKgVector3D m_CameraPos;
	CKhuGleSignal texture;
	CKhuGleSignal texture2;

	CKhuGle3DSprite(int nW, int nH, double Fov, double Far, double Near, KgColor24 fgColor);
	~CKhuGle3DSprite();

	static void DrawTriangle(unsigned char** R, unsigned char** G, unsigned char** B, int nW, int nH,
		int x0, int y0, int x1, int y1, int x2, int y2, KgColor24 Color24);
	static void DrawTriangleMesh(unsigned char** R, unsigned char** G, unsigned char** B, int nW, int nH,
		CKgTriangle& tri, CKgTriangle& normal, CKgTriangle& uv, CKhuGleSignal& texture, CKhuGleSignal& texture2,
		double** ZBuffer, KgColor24 Color24);
	static void MatrixVector44(CKgVector3D& out, CKgVector3D v, double** M);
	static double** ComputeViewMatrix(CKgVector3D Camera, CKgVector3D Target, CKgVector3D CameraUp);

	void Render();
	void MoveBy(double OffsetX, double OffsetY, double OffsetZ);
};

CKhuGle3DSprite::CKhuGle3DSprite(int nW, int nH, double Fov, double Far, double Near, KgColor24 fgColor) {
	m_fgColor = fgColor;
	m_CameraPos = CKgVector3D(0., -0.2, -2);

	m_ProjectionMatrix = dmatrix(4, 4);
	for (int r = 0; r < 4; ++r)
		for (int c = 0; c < 4; ++c)
			m_ProjectionMatrix[r][c] = 0.;

	m_ProjectionMatrix[0][0] = (double)nH / (double)nW * 1. / tan(Fov / 2.);
	m_ProjectionMatrix[1][1] = 1. / tan(Fov / 2.);
	m_ProjectionMatrix[2][2] = (-Near - Far) / (Near - Far);
	m_ProjectionMatrix[2][3] = 2. * (Far * Near) / (Near - Far);
	m_ProjectionMatrix[3][2] = 1.;
	m_ProjectionMatrix[3][3] = 0.;

	auto loader = ObjLoader();
	loader.LoadObjFromFile("cube.obj");

	for (auto tri : loader.faces) {
		SurfaceMesh.push_back(CKgTriangle(loader.vertexs[std::get<0>(tri)], loader.vertexs[std::get<1>(tri)], loader.vertexs[std::get<2>(tri)]));
	}
	for (auto tri : loader.faceNorms) {
		Normal.push_back(CKgTriangle(loader.normals[std::get<0>(tri)], loader.normals[std::get<1>(tri)], loader.normals[std::get<2>(tri)]));
	}
	for (auto tri : loader.faceUvs) {
		Uv.push_back(CKgTriangle(loader.uvs[std::get<0>(tri)], loader.uvs[std::get<1>(tri)], loader.uvs[std::get<2>(tri)]));
	}

	texture.ReadBmp("texture.bmp");
};

CKhuGle3DSprite::~CKhuGle3DSprite() {
	free_dmatrix(m_ProjectionMatrix, 4, 4);
};

void CKhuGle3DSprite::DrawTriangle(unsigned char** R, unsigned char** G, unsigned char** B, int nW, int nH,
	int x0, int y0, int x1, int y1, int x2, int y2, KgColor24 Color24)
{
	CKhuGleSprite::DrawLine(R, G, B, nW, nH, x0, y0, x1, y1, Color24);
	CKhuGleSprite::DrawLine(R, G, B, nW, nH, x1, y1, x2, y2, Color24);
	CKhuGleSprite::DrawLine(R, G, B, nW, nH, x2, y2, x0, y0, Color24);
}

void CKhuGle3DSprite::DrawTriangleMesh(unsigned char** R, unsigned char** G, unsigned char** B, int nW, int nH,
	CKgTriangle& tri, CKgTriangle& normal, CKgTriangle& uv, CKhuGleSignal& texture, CKhuGleSignal& texture2, double** ZBuffer, KgColor24 Color24)
{
	// 레스터화

	// 범위 지정
	int recStartX = std::clamp(min(min(tri.v0.x, tri.v1.x), tri.v2.x) - 1, 0., 10000.);
	int recStartY = std::clamp(min(min(tri.v0.y, tri.v1.y), tri.v2.y) - 1, 0., 10000.);
	int recEndX = std::clamp(max(max(tri.v0.x, tri.v1.x), tri.v2.x) + 1, 0., 10000.);
	int recEndY = std::clamp(max(max(tri.v0.y, tri.v1.y), tri.v2.y) + 1, 0., 10000.);

	for (int x = recStartX; x < recEndX; x++)
		for (int y = recStartY; y < recEndY; y++)
		{
			// x2 체커보드 렌더링
			if (X2CHECKORBOARD == true)
				if (!(((x + 1) % 4 / 2 == 0) != ((y + 1) % 4 / 2 == 0)))
					continue;

			// x2 선형 업스케일 렌더링
			if (X2UPSCALE)
				if (x % 2 != 0 || y % 2 != 0)
					continue;

			if (X4BLINEAR)
				if (x % 4 != 0 || y % 4 != 0)
					continue;

			if (X4PIXELSCALE)
				if (x % 4 != 0 || y % 4 != 0)
					continue;

			if (X4CUBIC)
				if (x % 4 != 0 || y % 4 != 0)
					continue;

			// 화면 나가는 픽셀 제거
			if ((x >= nW || x < 0) || (y >= nH || y < 0))
				continue;

			CKgVector3D point(x, y, 0);

			CKgVector3D v0(tri.v0.x, tri.v0.y, 0);
			CKgVector3D v1(tri.v1.x, tri.v1.y, 0);
			CKgVector3D v2(tri.v2.x, tri.v2.y, 0);

			CKgVector3D v0tov1 = v1 - v0;
			CKgVector3D v0tov2 = v2 - v0;
			CKgVector3D v0toP = point - v0;
			CKgVector3D v1tov2 = v2 - v1;
			CKgVector3D v2tov0 = v0 - v2;

			// 삼각형 내부의 점 찾기
			if (!(v0tov1.x * (point - v0).y - v0tov1.y * (point - v0).x <= 0 && v1tov2.x * (point - v1).y - v1tov2.y * (point - v1).x <= 0 && v2tov0.x * (point - v2).y - v2tov0.y * (point - v2).x <= 0))
				continue;

			// 버텍스 값 보간
			double b = (v0toP.y * v0tov2.x - v0tov2.y * v0toP.x) / (v0tov2.x * v0tov1.y - v0tov2.y * v0tov1.x);
			double a = (v0toP.x - v0tov1.x * b) / v0tov2.x;
			double v0value = 1 - a - b;
			double v1value = b;
			double v2value = a;

			// 깊이 값 테스트
			double z = (tri.v0.z * v0value + tri.v1.z * v1value + tri.v2.z * v2value);
			if (ZBuffer[y][x] <= z)
				continue;
			else if (z <= 0)
				continue;
			ZBuffer[y][x] = z;

			// 여기서부터 셰이더 부분
			CKgVector3D normalvec = v0value * normal.v0 + v1value * normal.v1 + v2value * normal.v2;
			CKgVector3D uvVec = v0value * uv.v0 + v1value * uv.v1 + v2value * uv.v2;

			double dot = normalvec.Dot(CKgVector3D(1, 1, 1));
			dot = min(max(dot + 0.5, 0), 1.0);

			KgColor24 color = 0xFFFFFF;
			color = texture.ReadTexture(uvVec.x, uvVec.y);

			//color = KG_COLOR_24_RGB((unsigned char)255 * dot, (unsigned char)255 * dot, (unsigned char)255 * dot);

			R[y][x] = KgGetRed(color) * dot;
			G[y][x] = KgGetGreen(color) * dot;
			B[y][x] = KgGetBlue(color) * dot;

			renderPixel++;
		}
}

void CKhuGle3DSprite::MatrixVector44(CKgVector3D& out, CKgVector3D v, double** M)
{
	out.x = v.x * M[0][0] + v.y * M[0][1] + v.z * M[0][2] + M[0][3];
	out.y = v.x * M[1][0] + v.y * M[1][1] + v.z * M[1][2] + M[1][3];
	out.z = v.x * M[2][0] + v.y * M[2][1] + v.z * M[2][2] + M[2][3];

	double w = v.x * M[3][0] + v.y * M[3][1] + v.z * M[3][2] + M[3][3];

	if (fabs(w) > 0)
		out = (1. / w) * out;
}

double** CKhuGle3DSprite::ComputeViewMatrix(CKgVector3D Camera, CKgVector3D Target, CKgVector3D CameraUp)
{
	CKgVector3D Forward = Target - Camera;
	Forward.Normalize();
	CameraUp.Normalize();
	CKgVector3D Right = CameraUp.Cross(Forward);
	CKgVector3D Up = Forward.Cross(Right);

	double** RT = dmatrix(4, 4);
	double** View = dmatrix(4, 4);

	RT[0][0] = Right.x;
	RT[1][0] = Right.y;
	RT[2][0] = Right.z;
	RT[3][0] = 0.;
	RT[0][1] = Up.x;
	RT[1][1] = Up.y;
	RT[2][1] = Up.z;
	RT[3][1] = 0.;
	RT[0][2] = Forward.x;
	RT[1][2] = Forward.y;
	RT[2][2] = Forward.z;
	RT[3][2] = 0.;
	RT[0][3] = Camera.x;
	RT[1][3] = Camera.y;
	RT[2][3] = Camera.z;
	RT[3][3] = 1.;

	bool bInverse = InverseMatrix(RT, View, 4);

	free_dmatrix(RT, 4, 4);

	if (bInverse)
		return View;

	return nullptr;

}

void CKhuGle3DSprite::Render()
{
	if (!m_Parent) return;
	if (ROTATE) {
		double NewX = m_CameraPos.x * cos(Pi / 50.) - m_CameraPos.z * sin(Pi / 50.);
		double NewZ = m_CameraPos.x * sin(Pi / 50.) + m_CameraPos.z * cos(Pi / 50.);
		m_CameraPos.x = NewX;
		m_CameraPos.z = NewZ;
	}
	CKhuGleLayer* Parent = (CKhuGleLayer*)m_Parent;

	double** ViewMatrix = ComputeViewMatrix(m_CameraPos, CKgVector3D(0., 0., 0.), CKgVector3D(0., 1., 0.));
	if (ViewMatrix == nullptr) return;

	double** ZBuffer = dmatrix(Parent->m_nH, Parent->m_nW);
	for (int i = 0; i < Parent->m_nH; i++)
		for (int j = 0; j < Parent->m_nW; j++)
			ZBuffer[i][j] = 100;

	for (int i = 0; i < SurfaceMesh.size(); i++)
	{
		auto& Triangle = SurfaceMesh[i];
		auto& VertexNormal = Normal[i];
		auto& VertexUv = Uv[i];

		CKgVector3D Side01, Side02, Normal;

		Side01 = Triangle.v1 - Triangle.v0;
		Side02 = Triangle.v2 - Triangle.v1;

		Normal = Side01.Cross(Side02);
		Normal.Normalize();

		CKgTriangle ViewTriangle;
		CKgTriangle Projected;

		if (Normal.Dot(Triangle.v0 - m_CameraPos) < 0.) // 벡 페이스 컬링
		{
			renderTriangle++;

			MatrixVector44(ViewTriangle.v0, Triangle.v0, ViewMatrix);
			MatrixVector44(ViewTriangle.v1, Triangle.v1, ViewMatrix);
			MatrixVector44(ViewTriangle.v2, Triangle.v2, ViewMatrix);

			MatrixVector44(Projected.v0, ViewTriangle.v0, m_ProjectionMatrix);
			MatrixVector44(Projected.v1, ViewTriangle.v1, m_ProjectionMatrix);
			MatrixVector44(Projected.v2, ViewTriangle.v2, m_ProjectionMatrix);

			// 점을 중앙으로 이동
			Projected.v0.x += 1.;
			Projected.v0.y += 1.;
			Projected.v1.x += 1.;
			Projected.v1.y += 1.;
			Projected.v2.x += 1.;
			Projected.v2.y += 1.;
			Projected.v0.x *= Parent->m_nW / 2.;
			Projected.v0.y *= Parent->m_nH / 2.;
			Projected.v1.x *= Parent->m_nW / 2.;
			Projected.v1.y *= Parent->m_nH / 2.;
			Projected.v2.x *= Parent->m_nW / 2.;
			Projected.v2.y *= Parent->m_nH / 2.;
			Projected.v0.x -= 1.;
			Projected.v0.y -= 1.;
			Projected.v1.x -= 1.;
			Projected.v1.y -= 1.;
			Projected.v2.x -= 1.;
			Projected.v2.y -= 1.;

			// Draw Mesh
			DrawTriangleMesh(Parent->m_ImageR, Parent->m_ImageG, Parent->m_ImageB,
				Parent->m_nW, Parent->m_nH,
				Projected, VertexNormal, VertexUv, texture, texture2, ZBuffer, m_fgColor);

			// Draw Normal Line
			if (NORMALVISUAL) {
				CKgVector3D normStart = (Triangle.v0 + Triangle.v1 + Triangle.v2) / 3;
				CKgVector3D normEnd = normStart + 0.1 * Normal;
				CKgTriangle viewNorm, projectedNorm;
				MatrixVector44(viewNorm.v0, normStart, ViewMatrix);
				MatrixVector44(viewNorm.v1, normEnd, ViewMatrix);
				MatrixVector44(projectedNorm.v0, viewNorm.v0, m_ProjectionMatrix);
				MatrixVector44(projectedNorm.v1, viewNorm.v1, m_ProjectionMatrix);
				projectedNorm.v0.x += 1.;
				projectedNorm.v0.y += 1.;
				projectedNorm.v1.x += 1.;
				projectedNorm.v1.y += 1.;
				projectedNorm.v0.x *= Parent->m_nW / 2;
				projectedNorm.v0.y *= Parent->m_nH / 2;
				projectedNorm.v1.x *= Parent->m_nW / 2;
				projectedNorm.v1.y *= Parent->m_nH / 2;
				projectedNorm.v0.x -= 1.;
				projectedNorm.v0.y -= 1.;
				projectedNorm.v1.x -= 1.;
				projectedNorm.v1.y -= 1.;
				DrawLine(Parent->m_ImageR, Parent->m_ImageG, Parent->m_ImageB,
					Parent->m_nW, Parent->m_nH, projectedNorm.v0.x, projectedNorm.v0.y, projectedNorm.v1.x, projectedNorm.v1.y, m_fgColor);
			}

			// Draw Edge
			if (EDGEVISUAL)
				DrawTriangle(Parent->m_ImageR, Parent->m_ImageG, Parent->m_ImageB,
					Parent->m_nW, Parent->m_nH,
					(int)Projected.v0.x, (int)Projected.v0.y,
					(int)Projected.v1.x, (int)Projected.v1.y,
					(int)Projected.v2.x, (int)Projected.v2.y, m_fgColor);
		}
	}

	// 후처리
	if (DRAWCLIFF) {
		if (X2CHECKORBOARD) {
			for (int y = 0; y < Parent->m_nH / 4; y++)
				for (int x = 0; x < Parent->m_nW / 4; x++) {
					x *= 4; y *= 4;
					if (x % 4 == 0 && y % 4 == 0) {
						Parent->m_ImageR[y][x] = (Parent->m_ImageR[y + 1][x] + Parent->m_ImageR[y][x + 1]) / 2;
						Parent->m_ImageR[y + 1][x + 1] = Parent->m_ImageR[y][x];
						Parent->m_ImageG[y][x] = (Parent->m_ImageG[y + 1][x] + Parent->m_ImageG[y][x + 1]) / 2;
						Parent->m_ImageG[y + 1][x + 1] = Parent->m_ImageG[y][x];
						Parent->m_ImageB[y][x] = (Parent->m_ImageB[y + 1][x] + Parent->m_ImageB[y][x + 1]) / 2;
						Parent->m_ImageB[y + 1][x + 1] = Parent->m_ImageB[y][x];
					}
					y += 3;
					if (x % 4 == 0 && y % 4 == 3) {
						Parent->m_ImageR[y][x] = (Parent->m_ImageR[y - 1][x] + Parent->m_ImageR[y][x + 1]) / 2;
						Parent->m_ImageR[y - 1][x + 1] = Parent->m_ImageR[y][x];
						Parent->m_ImageG[y][x] = (Parent->m_ImageG[y - 1][x] + Parent->m_ImageG[y][x + 1]) / 2;
						Parent->m_ImageG[y - 1][x + 1] = Parent->m_ImageG[y][x];
						Parent->m_ImageB[y][x] = (Parent->m_ImageB[y - 1][x] + Parent->m_ImageB[y][x + 1]) / 2;
						Parent->m_ImageB[y - 1][x + 1] = Parent->m_ImageB[y][x];
					}
					y -= 3; x += 3;
					if (x % 4 == 3 && y % 4 == 0) {
						Parent->m_ImageR[y][x] = (Parent->m_ImageR[y + 1][x] + Parent->m_ImageR[y][x - 1]) / 2;
						Parent->m_ImageR[y + 1][x - 1] = Parent->m_ImageR[y][x];
						Parent->m_ImageG[y][x] = (Parent->m_ImageG[y + 1][x] + Parent->m_ImageG[y][x - 1]) / 2;
						Parent->m_ImageG[y + 1][x - 1] = Parent->m_ImageG[y][x];
						Parent->m_ImageB[y][x] = (Parent->m_ImageB[y + 1][x] + Parent->m_ImageB[y][x - 1]) / 2;
						Parent->m_ImageB[y + 1][x - 1] = Parent->m_ImageB[y][x];
					}
					y += 3;
					if (x % 4 == 3 && y % 4 == 3) {
						Parent->m_ImageR[y][x] = (Parent->m_ImageR[y - 1][x] + Parent->m_ImageR[y][x - 1]) / 2;
						Parent->m_ImageR[y - 1][x - 1] = Parent->m_ImageR[y][x];
						Parent->m_ImageG[y][x] = (Parent->m_ImageG[y - 1][x] + Parent->m_ImageG[y][x - 1]) / 2;
						Parent->m_ImageG[y - 1][x - 1] = Parent->m_ImageG[y][x];
						Parent->m_ImageB[y][x] = (Parent->m_ImageB[y - 1][x] + Parent->m_ImageB[y][x - 1]) / 2;
						Parent->m_ImageB[y - 1][x - 1] = Parent->m_ImageB[y][x];
					}
					x -= 3; y -= 3; x /= 4; y /= 4;
				}
		}

		if (X2UPSCALE) {
			for (int y = 0; y < Parent->m_nH / 2 - 1; y++)
				for (int x = 0; x < Parent->m_nW / 2 - 1; x++) {
					x *= 2; y *= 2;
					Parent->m_ImageR[y][x + 1] = (Parent->m_ImageR[y][x] + Parent->m_ImageR[y][x + 2]) / 2;
					Parent->m_ImageG[y][x + 1] = (Parent->m_ImageG[y][x] + Parent->m_ImageG[y][x + 2]) / 2;
					Parent->m_ImageB[y][x + 1] = (Parent->m_ImageB[y][x] + Parent->m_ImageB[y][x + 2]) / 2;

					Parent->m_ImageR[y + 1][x] = (Parent->m_ImageR[y][x] + Parent->m_ImageR[y + 2][x]) / 2;
					Parent->m_ImageG[y + 1][x] = (Parent->m_ImageG[y][x] + Parent->m_ImageG[y + 2][x]) / 2;
					Parent->m_ImageB[y + 1][x] = (Parent->m_ImageB[y][x] + Parent->m_ImageB[y + 2][x]) / 2;

					Parent->m_ImageR[y + 1][x + 1] = (Parent->m_ImageR[y][x] + Parent->m_ImageR[y + 2][x + 2] + Parent->m_ImageR[y + 2][x] + Parent->m_ImageR[y + 2][x]) / 4;
					Parent->m_ImageG[y + 1][x + 1] = (Parent->m_ImageG[y][x] + Parent->m_ImageG[y + 2][x + 2] + Parent->m_ImageG[y + 2][x] + Parent->m_ImageG[y + 2][x]) / 4;
					Parent->m_ImageB[y + 1][x + 1] = (Parent->m_ImageB[y][x] + Parent->m_ImageB[y + 2][x + 2] + Parent->m_ImageB[y + 2][x] + Parent->m_ImageB[y + 2][x]) / 4;
					x /= 2; y /= 2;
				}
		}

		if (X4BLINEAR) {
			for (int y = 1; y < Parent->m_nH / 4 - 1; y++)
				for (int x = 1; x < Parent->m_nW / 4 - 1; x++) {
					x *= 4; y *= 4;
					for (int yy = 0; yy < 4; yy++)
						for (int xx = 0; xx < 4; xx++) {
							if (yy == 0 && xx == 0)
								continue;
							Parent->m_ImageR[y + yy][x + xx] = ((Parent->m_ImageR[y][x + 4] * xx + Parent->m_ImageR[y][x] * (4 - xx)) * (4 - yy) + (Parent->m_ImageR[y + 4][x + 4] * xx + Parent->m_ImageR[y + 4][x] * (4 - xx)) * (yy)) / 16;
							Parent->m_ImageG[y + yy][x + xx] = ((Parent->m_ImageG[y][x + 4] * xx + Parent->m_ImageG[y][x] * (4 - xx)) * (4 - yy) + (Parent->m_ImageG[y + 4][x + 4] * xx + Parent->m_ImageG[y + 4][x] * (4 - xx)) * (yy)) / 16;
							Parent->m_ImageB[y + yy][x + xx] = ((Parent->m_ImageB[y][x + 4] * xx + Parent->m_ImageB[y][x] * (4 - xx)) * (4 - yy) + (Parent->m_ImageB[y + 4][x + 4] * xx + Parent->m_ImageB[y + 4][x] * (4 - xx)) * (yy)) / 16;
						}
					x /= 4; y /= 4;
				}
		}

		if (X4PIXELSCALE) {
			for (int y = 0; y < Parent->m_nH / 4 - 1; y++)
				for (int x = 0; x < Parent->m_nW / 4 - 1; x++) {
					x *= 4; y *= 4;
					for (int yy = 0; yy < 4; yy++)
						for (int xx = 0; xx < 4; xx++) {
							if (yy == 0 && xx == 0)
								continue;
							Parent->m_ImageR[y + yy][x + xx] = Parent->m_ImageR[y][x];
							Parent->m_ImageG[y + yy][x + xx] = Parent->m_ImageG[y][x];
							Parent->m_ImageB[y + yy][x + xx] = Parent->m_ImageB[y][x];
						}
					x /= 4; y /= 4;
				}
		}

		if (X4CUBIC) {
			for (int y = 1; y < Parent->m_nH / 4 - 2; y++)
				for (int x = 1; x < Parent->m_nW / 4 - 2; x++) {
					x *= 4; y *= 4;
					for (int yy = 0; yy < 4; yy++)
						for (int xx = 0; xx < 4; xx++) {
							if (yy == 0 && xx == 0)
								continue;

							double r1 = (3 * (4 + xx) * Parent->m_ImageR[y - 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageR[y - 4][x + 4] * (8 - xx)
								- Parent->m_ImageR[y - 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageR[y - 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double r2 = (3 * (4 + xx) * Parent->m_ImageR[y][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageR[y][x + 4] * (8 - xx)
								- Parent->m_ImageR[y][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageR[y][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double r3 = (3 * (4 + xx) * Parent->m_ImageR[y + 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageR[y + 4][x + 4] * (8 - xx)
								- Parent->m_ImageR[y + 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageR[y + 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double r4 = (3 * (4 + xx) * Parent->m_ImageR[y + 8][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageR[y + 8][x + 4] * (8 - xx)
								- Parent->m_ImageR[y + 8][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageR[y + 8][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double r = (3 * (4 + yy) * r2 * (4 - yy) * (8 - yy)
								+ 3 * (4 + yy) * yy * r3 * (8 - yy)
								- r1 * yy * (4 - yy) * (8 - yy)
								- r3 * yy * (4 - yy) * (4 + yy)) / (6 * 4 * 4 * 4);
							Parent->m_ImageR[y + yy][x + xx] = std::clamp(r, 0., 255.);

							double g1 = (3 * (4 + xx) * Parent->m_ImageG[y - 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageG[y - 4][x + 4] * (8 - xx)
								- Parent->m_ImageG[y - 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageG[y - 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double g2 = (3 * (4 + xx) * Parent->m_ImageG[y][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageG[y][x + 4] * (8 - xx)
								- Parent->m_ImageG[y][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageG[y][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double g3 = (3 * (4 + xx) * Parent->m_ImageG[y + 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageG[y + 4][x + 4] * (8 - xx)
								- Parent->m_ImageG[y + 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageG[y + 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double g4 = (3 * (4 + xx) * Parent->m_ImageG[y + 8][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageG[y + 8][x + 4] * (8 - xx)
								- Parent->m_ImageG[y + 8][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageG[y + 8][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double g = (3 * (4 + yy) * g2 * (4 - yy) * (8 - yy)
								+ 3 * (4 + yy) * yy * g3 * (8 - yy)
								- g1 * yy * (4 - yy) * (8 - yy)
								- g3 * yy * (4 - yy) * (4 + yy)) / (6 * 4 * 4 * 4);
							Parent->m_ImageG[y + yy][x + xx] = std::clamp(g, 0., 255.);

							double b1 = (3 * (4 + xx) * Parent->m_ImageB[y - 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageB[y - 4][x + 4] * (8 - xx)
								- Parent->m_ImageB[y - 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageB[y - 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double b2 = (3 * (4 + xx) * Parent->m_ImageB[y][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageB[y][x + 4] * (8 - xx)
								- Parent->m_ImageB[y][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageB[y][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double b3 = (3 * (4 + xx) * Parent->m_ImageB[y + 4][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageB[y + 4][x + 4] * (8 - xx)
								- Parent->m_ImageB[y + 4][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageB[y + 4][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double b4 = (3 * (4 + xx) * Parent->m_ImageB[y + 8][x] * (4 - xx) * (8 - xx)
								+ 3 * (4 + xx) * xx * Parent->m_ImageB[y + 8][x + 4] * (8 - xx)
								- Parent->m_ImageB[y + 8][x - 4] * xx * (4 - xx) * (8 - xx)
								- Parent->m_ImageB[y + 8][x + 8] * xx * (4 - xx) * (4 + xx)) / (6 * 4 * 4 * 4);
							double b = (3 * (4 + yy) * b2 * (4 - yy) * (8 - yy)
								+ 3 * (4 + yy) * yy * b3 * (8 - yy)
								- b1 * yy * (4 - yy) * (8 - yy)
								- b3 * yy * (4 - yy) * (4 + yy)) / (6 * 4 * 4 * 4);
							Parent->m_ImageB[y + yy][x + xx] = std::clamp(b, 0., 255.);
						}
					x /= 4; y /= 4;
				}
		}
	}

	free_dmatrix(ViewMatrix, 4, 4);
	free_dmatrix(ZBuffer, Parent->m_nH, Parent->m_nW);
}

void CKhuGle3DSprite::MoveBy(double OffsetX, double OffsetY, double OffsetZ)
{
	for (auto& Triangle : SurfaceMesh)
	{
		Triangle.v0 = Triangle.v0 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
		Triangle.v1 = Triangle.v1 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
		Triangle.v2 = Triangle.v2 + CKgVector3D(OffsetX, OffsetY, OffsetZ);
	}
}

class CThreeDim : public CKhuGleWin
{
public:
	CKhuGleLayer* m_pGameLayer;

	CKhuGle3DSprite* m_pObject3D;

	CThreeDim(int nW, int nH);
	void Update();

	CKgPoint m_LButtonStart, m_LButtonEnd;
	int m_nLButtonStatus;
};

CThreeDim::CThreeDim(int nW, int nH) : CKhuGleWin(nW, nH)
{
	m_nLButtonStatus = 0;

	m_Gravity = CKgVector2D(0., 98.);
	m_AirResistance = CKgVector2D(0.1, 0.1);

	m_pScene = new CKhuGleScene(1100, 1100, KG_COLOR_24_RGB(100, 100, 150));

	m_pGameLayer = new CKhuGleLayer(1000, 1000, KG_COLOR_24_RGB(150, 150, 200), CKgPoint(50, 50));
	m_pScene->AddChild(m_pGameLayer);

	m_pObject3D = new CKhuGle3DSprite(m_pGameLayer->m_nW, m_pGameLayer->m_nH, Pi / 2., 1000, 0.1, KG_COLOR_24_RGB(255, 255, 255));
	m_pObject3D->MoveBy(0, 1, 0);

	m_pGameLayer->AddChild(m_pObject3D);
}

void CThreeDim::Update()
{
	if (m_bKeyPressed['O'] || m_bKeyPressed['C'] || m_bKeyPressed['X'] || m_bKeyPressed['F'] || m_bKeyPressed['P'] || m_bKeyPressed['B']) {
		X2CHECKORBOARD = false;
		X2UPSCALE = false;
		X4BLINEAR = false;
		X4PIXELSCALE = false;
		X4CUBIC = false;

		if (m_bKeyPressed['C'])
			X2CHECKORBOARD = true;
		else if (m_bKeyPressed['X'])
			X2UPSCALE = true;
		else if (m_bKeyPressed['F'])
			X4BLINEAR = true;
		else if (m_bKeyPressed['P'])
			X4PIXELSCALE = true;
		else if (m_bKeyPressed['B'])
			X4CUBIC = true;

		m_bKeyPressed['X'] = false;
		m_bKeyPressed['C'] = false;
		m_bKeyPressed['O'] = false;
		m_bKeyPressed['F'] = false;
		m_bKeyPressed['P'] = false;
		m_bKeyPressed['B'] = false;
	}
	if (m_bKeyPressed['N']) {
		NORMALVISUAL = !NORMALVISUAL;
		m_bKeyPressed['N'] = false;
	}
	if (m_bKeyPressed['E']) {
		EDGEVISUAL = !EDGEVISUAL;
		m_bKeyPressed['E'] = false;
	}
	if (m_bKeyPressed['R']) {
		ROTATE = !ROTATE;
		m_bKeyPressed['R'] = false;
	}
	if (m_bKeyPressed['Z']) {
		DRAWCLIFF = !DRAWCLIFF;
		m_bKeyPressed['Z'] = false;
	}

	m_pScene->Render();
	std::string fps = "FPS:" + std::to_string(m_Fps) + "  DrawTriangle: " + std::to_string(renderTriangle) + "  DrawPixel: " + std::to_string(renderPixel);
	renderTriangle = 0;
	renderPixel = 0;
	
	DrawSceneTextPos(fps.c_str(), CKgPoint(0, 0));

	CKhuGleWin::Update();
}

int main()
{
	std::cout << "O:Original  C:x2checkerBoard   X:x2linear   P:x4Nearest   F:x4linear  B:x4cubic   \nN:normal   E:edge  Z:onlyRenderReal  R:Rotate" << std::endl;

	CThreeDim* pThreeDim = new CThreeDim(1100, 1100);

	KhuGleWinInit(pThreeDim);

	return 0;
}


