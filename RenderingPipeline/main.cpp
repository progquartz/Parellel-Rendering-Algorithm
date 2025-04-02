
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Window.hpp>
#include <sstream>
#include <fstream>
#include <list>
#include <omp.h>
using namespace std;

//Declare ini
struct RenderSettings
{
	bool Watermark;
	bool Wireframe;
	int xres;
	int yres;
	int depth;
	float FOV;
	float xsens;
	float ysens;
	sf::Color DefaultText;
	uint32_t WindowMode;
};

//Declare 3D Vector Format
struct vec3
{
	float x = 0, y = 0, z = 0, w = 1;
};

//Declare 3D Polygon Format
struct tri3
{
	vec3 p[3];
	int color;
};

//Declare 3D Mesh Format
struct mesh
{
	vector<tri3> polys;

	bool LoadObj(string Filename)
	{
		ifstream file(Filename);
		if (!file.is_open())
			return false;

		vector<vec3> vertices;

		while (!file.eof())
		{
			char line[128];
			file.getline(line, 128);

			std::stringstream stream;
			stream << line;

			char junk;
			if (line[0] == 'v')
			{
				vec3 v;
				stream >> junk >> v.x >> v.y >> v.z;
				vertices.push_back(v);
			}
			if (line[0] == 'f')
			{
				int f[3];
				stream >> junk >> f[0] >> f[1] >> f[2];
				polys.push_back({ vertices[f[0] - 1], vertices[f[1] - 1], vertices[f[2] - 1] });
			}

		}

		return true;
	}

};

//Declare 4x4 Matrix Format
struct mat4
{
	float m[4][4] = { 0 };
};

//DrawMesh
vector<tri3> DrawMesh(mesh obj, mat4 Projection, mat4 ViewMatrix, mat4 World, vec3 Camera);

//NomalVector
vec3 NomalizingVector3(vec3 vector1, vec3 vector2, vec3 vector3);

//RasterMesh
void RasterRenderMesh(vector<tri3> TriVector, RenderSettings ini, sf::RenderWindow& mywindow);


void CheckAllEvent(sf::RenderWindow* mywindow, RenderSettings* ini, sf::Clock* clock, vec3* Camera, float* Runtime, float* Yaw, float* Pitch, vec3* lookdir, sf::Color sky, sf::Time* delta);

//Function for Multiplying Vectors by Matrix
vec3 MatxVec(mat4& m, vec3& i)
{
	vec3 v;
	v.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + i.w * m.m[3][0];
	v.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + i.w * m.m[3][1];
	v.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + i.w * m.m[3][2];
	v.w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + i.w * m.m[3][3];
	return v;
}

//Functions for Manipulating Vectors
vec3 AddVec(vec3& v1, vec3& v2)
{
	return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
}
vec3 SubVec(vec3& v1, vec3& v2)
{
	return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
}

// 벡터에 스칼라를 곱하는 함수.
vec3 VecxScalar(vec3& v1, float k)
{
	return { v1.x * k, v1.y * k, v1.z * k };
}

// 벡터에 스칼라를 나누는 함수.
vec3 VecdScalar(vec3& v1, float k)
{
	return { v1.x / k, v1.y / k, v1.z / k };
}

float Dot(vec3& v1, vec3& v2)
{
	return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}
float Magnitude(vec3& v)
{
	return sqrtf(Dot(v, v));
}
vec3 Norm(vec3& v)
{
	float l = Magnitude(v);
	return { v.x / l, v.y / l, v.z / l };
}
vec3 Cross(vec3& v1, vec3& v2)
{
	vec3 v3;
	v3.x = v1.y * v2.z - v1.z * v2.y;
	v3.y = v1.z * v2.x - v1.x * v2.z;
	v3.z = v1.x * v2.y - v1.y * v2.x;
	return v3;
}
vec3 VectorIntersectPlane(vec3& plane_p, vec3& plane_n, vec3& lineStart, vec3& lineEnd)
{
	plane_n = Norm(plane_n);
	float plane_d = -Dot(plane_n, plane_p);
	float ad = Dot(lineStart, plane_n);
	float bd = Dot(lineEnd, plane_n);
	float t = (-plane_d - ad) / (bd - ad);
	vec3 lineStartToEnd = SubVec(lineEnd, lineStart);
	vec3 lineToIntersect = VecxScalar(lineStartToEnd, t);
	return AddVec(lineStart, lineToIntersect);
}

int ClipTriangleAgainstPlane(vec3 plane_p, vec3 plane_n, tri3& in_tri, tri3& out_tri1, tri3& out_tri2)
{
	plane_n = Norm(plane_n); //Make sure plane normal is unit vector

	auto dist = [&](vec3& p)
	{
		vec3 n = Norm(p);
		return (plane_n.x * p.x + plane_n.y * p.y + plane_n.z * p.z - Dot(plane_n, plane_p)); //Return signed distance from point to plane
	};

	//Designate points as in plane or out of plane based on whether signed distance is positive or negative
	vec3* inside_points[3];  int nInsidePointCount = 0;
	vec3* outside_points[3]; int nOutsidePointCount = 0;

	float d0 = dist(in_tri.p[0]);
	float d1 = dist(in_tri.p[1]);
	float d2 = dist(in_tri.p[2]);

	if (d0 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[0]; }
	else { outside_points[nOutsidePointCount++] = &in_tri.p[0]; }
	if (d1 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[1]; }
	else { outside_points[nOutsidePointCount++] = &in_tri.p[1]; }
	if (d2 >= 0) { inside_points[nInsidePointCount++] = &in_tri.p[2]; }
	else { outside_points[nOutsidePointCount++] = &in_tri.p[2]; }

	if (nInsidePointCount == 0)
	{
		return 0; //Triangle is completely outside of plane, destroy it
	}

	if (nInsidePointCount == 3)
	{
		out_tri1 = in_tri;

		return 1; //All three points are valid, return the original triangle
	}

	if (nInsidePointCount == 1 && nOutsidePointCount == 2)
	{
		out_tri1.color = in_tri.color;

		out_tri1.p[0] = *inside_points[0];

		out_tri1.p[1] = VectorIntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);
		out_tri1.p[2] = VectorIntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[1]);

		return 1; //One point is valid, clip the triangle down to a new triangle and return the new triangle
	}

	if (nInsidePointCount == 2 && nOutsidePointCount == 1)
	{

		out_tri1.color = in_tri.color;

		out_tri2.color = in_tri.color;

		out_tri1.p[0] = *inside_points[0];
		out_tri1.p[1] = *inside_points[1];
		out_tri1.p[2] = VectorIntersectPlane(plane_p, plane_n, *inside_points[0], *outside_points[0]);

		out_tri2.p[0] = *inside_points[1];
		out_tri2.p[1] = out_tri1.p[2];
		out_tri2.p[2] = VectorIntersectPlane(plane_p, plane_n, *inside_points[1], *outside_points[0]);

		return 2; //Two points are valid, split valid area into two triangles and return those
	}
}

//Functions for Manipulating Matrices
mat4 IdentityMatrix()
{
	mat4 matrix;
	matrix.m[0][0] = 1.0f;
	matrix.m[1][1] = 1.0f;
	matrix.m[2][2] = 1.0f;
	matrix.m[3][3] = 1.0f;
	return matrix;
}
mat4 XRotationMatrix(float ThetaRad)
{
	mat4 matrix;
	matrix.m[0][0] = 1.0f;
	matrix.m[1][1] = cosf(ThetaRad);
	matrix.m[1][2] = sinf(ThetaRad);
	matrix.m[2][1] = -sinf(ThetaRad);
	matrix.m[2][2] = cosf(ThetaRad);
	matrix.m[3][3] = 1.0f;
	return matrix;
}
mat4 YRotationMatrix(float ThetaRad)
{
	mat4 matrix;
	matrix.m[0][0] = cosf(ThetaRad);
	matrix.m[0][2] = sinf(ThetaRad);
	matrix.m[2][0] = -sinf(ThetaRad);
	matrix.m[1][1] = 1.0f;
	matrix.m[2][2] = cosf(ThetaRad);
	matrix.m[3][3] = 1.0f;
	return matrix;
}
mat4 ZRotationMatrix(float ThetaRad)
{
	mat4 matrix;
	matrix.m[0][0] = cosf(ThetaRad);
	matrix.m[0][1] = sinf(ThetaRad);
	matrix.m[1][0] = -sinf(ThetaRad);
	matrix.m[1][1] = cosf(ThetaRad);
	matrix.m[2][2] = 1.0f;
	matrix.m[3][3] = 1.0f;
	return matrix;
}
mat4 TranslationMatrix(float x, float y, float z)
{
	mat4 matrix;
	matrix.m[0][0] = 1.0f;
	matrix.m[1][1] = 1.0f;
	matrix.m[2][2] = 1.0f;
	matrix.m[3][3] = 1.0f;
	matrix.m[3][0] = x;
	matrix.m[3][1] = y;
	matrix.m[3][2] = z;
	return matrix;
}
mat4 ProjectionMatrix(float FOVdeg, float Aspect, float CameraDistance, float RenderDistance)
{
	float FOVrad = 1.0f / tanf(FOVdeg * 0.5f / 180.0f * 3.14159f);
	mat4 matrix;
	matrix.m[0][0] = Aspect * FOVrad;
	matrix.m[1][1] = FOVrad;
	matrix.m[2][2] = RenderDistance / (RenderDistance - CameraDistance);
	matrix.m[3][2] = (-RenderDistance * CameraDistance) / (RenderDistance - CameraDistance);
	matrix.m[2][3] = 1.0f;
	matrix.m[3][3] = 0.0f;
	return matrix;
}
mat4 MatrixMultiply(mat4& m1, mat4& m2)
{
	mat4 matrix;
	for (int c = 0; c < 4; c++)
		for (int r = 0; r < 4; r++)
			matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] + m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
	return matrix;
}
mat4 PointingMatrix(vec3& pos, vec3& target, vec3& up)
{
	//Calculate new forward direction
	vec3 newForward = SubVec(target, pos);
	newForward = Norm(newForward);

	//Calculate new Up direction
	vec3 a = VecxScalar(newForward, Dot(up, newForward));
	vec3 newUp = SubVec(up, a);
	newUp = Norm(newUp);

	//New Right direction is easy, its just cross product
	vec3 newRight = Cross(newUp, newForward);

	//Construct Dimensioning and Translation Matrix	
	mat4 matrix;
	matrix.m[0][0] = newRight.x;	matrix.m[0][1] = newRight.y;	matrix.m[0][2] = newRight.z;	matrix.m[0][3] = 0.0f;
	matrix.m[1][0] = newUp.x;		matrix.m[1][1] = newUp.y;		matrix.m[1][2] = newUp.z;		matrix.m[1][3] = 0.0f;
	matrix.m[2][0] = newForward.x;	matrix.m[2][1] = newForward.y;	matrix.m[2][2] = newForward.z;	matrix.m[2][3] = 0.0f;
	matrix.m[3][0] = pos.x;			matrix.m[3][1] = pos.y;			matrix.m[3][2] = pos.z;			matrix.m[3][3] = 1.0f;
	return matrix;

}
mat4 MatrixQuickInverse(mat4& m) //Only for Rotation/Translation Matrices
{
	mat4 matrix;
	matrix.m[0][0] = m.m[0][0]; matrix.m[0][1] = m.m[1][0]; matrix.m[0][2] = m.m[2][0]; matrix.m[0][3] = 0.0f;
	matrix.m[1][0] = m.m[0][1]; matrix.m[1][1] = m.m[1][1]; matrix.m[1][2] = m.m[2][1]; matrix.m[1][3] = 0.0f;
	matrix.m[2][0] = m.m[0][2]; matrix.m[2][1] = m.m[1][2]; matrix.m[2][2] = m.m[2][2]; matrix.m[2][3] = 0.0f;
	matrix.m[3][0] = -(m.m[3][0] * matrix.m[0][0] + m.m[3][1] * matrix.m[1][0] + m.m[3][2] * matrix.m[2][0]);
	matrix.m[3][1] = -(m.m[3][0] * matrix.m[0][1] + m.m[3][1] * matrix.m[1][1] + m.m[3][2] * matrix.m[2][1]);
	matrix.m[3][2] = -(m.m[3][0] * matrix.m[0][2] + m.m[3][1] * matrix.m[1][2] + m.m[3][2] * matrix.m[2][2]);
	matrix.m[3][3] = 1.0f;
	return matrix;
}

int main()
{
	//SETTINGS
	RenderSettings ini;
	ini.Watermark = true;
	ini.Wireframe = true;
	ini.xres = 1920;
	ini.yres = 1080;
	ini.depth = 32;
	ini.FOV = 105.0f;
	ini.xsens = 1.0f;
	ini.ysens = 1.0f;
	ini.DefaultText.r = 230;
	ini.DefaultText.g = 210;
	ini.DefaultText.b = 210;
	ini.DefaultText.a = 200;
	ini.WindowMode = sf::Style::Fullscreen;

	//VERSION NUMBER
	sf::String versionno = "ARTIFICE 0.02";
	sf::String watermark;
	sf::String sFPS;
	sf::String camerax;
	sf::String cameray;
	sf::String cameraz;
	sf::String camerayaw;
	int FPS;

	//Seed Random Number Generator
	srand(time(NULL));

	//Initialize Text
	sf::Font CourierNew;
	CourierNew.loadFromFile("CourierNew.ttf");
	sf::Text watermarktext;
	watermarktext.setPosition(10.0f, 10.0f);
	watermarktext.setFont(CourierNew);
	watermarktext.setCharacterSize(25);
	watermarktext.setFillColor(ini.DefaultText);

	//Initialize Sound

	//Initialize Graphics

	vec3 Camera = { 0, 60, -200 };


	float Yaw = 1.6f;
	float Pitch = 1.5f;
	vec3 lookdir; // 바라보는 방향 제작.
	mat4 Projection = ProjectionMatrix(ini.FOV, float(ini.yres) / float(ini.xres), 0.1f, 1000.0f);
	float Theta = 0.1f;

	//Define Monochrome Tint
	sf::Color sky(30, 30, 50, 255);

	//Initialize Gameplay
	//mesh obj;
	//obj.LoadObj("marbles000.obj");

	//Open Window
	sf::RenderWindow mywindow(sf::VideoMode(ini.xres, ini.yres), versionno, ini.WindowMode);

	//Begin Measuring Time
	sf::Clock clock;
	float Runtime = 0.0;
	vec3 Up = { 0,0,1 };
	sf::Time delta;

	int objNumCount = 0;

	//Run Program Until User Closes Window
	while (mywindow.isOpen())
	{
		
		// -> 애니메이션화.
		mesh obj;
		if (objNumCount >= 100)
		{
			obj.LoadObj("marbles_result/marbles" + to_string(objNumCount) + ".obj");
		}
		else if (objNumCount >= 10)
		{
			obj.LoadObj("marbles_result/marbles0" + to_string(objNumCount) + ".obj");
		}
		else
		{
			obj.LoadObj("marbles_result/marbles00" + to_string(objNumCount) + ".obj");
		}

		objNumCount++;

		if (objNumCount >= 200)
		{
			objNumCount = 0;
		}
		

		


		//Check All New Events
		CheckAllEvent(&mywindow, &ini, &clock, &Camera, &Runtime, &Yaw, &Pitch, &lookdir, sky, &delta);
		mat4 RotZ, RotY, RotX;
		Theta = 0.0f;

		//Define Camera Rotation Matrices
		RotZ = ZRotationMatrix(Theta);
		RotY = YRotationMatrix(Theta);
		RotX = XRotationMatrix(Theta);


		//Move Triangles Away from Camera
		mat4 Translation;
		Translation = TranslationMatrix(5.0f, 0.0f, 0.0f);

		mat4 World;
		World = IdentityMatrix();
		World = MatrixMultiply(RotZ, RotX);
		World = MatrixMultiply(World, Translation);
		vec3 Target = { 1,0,0 };
		mat4 CameraRotation = ZRotationMatrix(-Yaw);
		mat4 PitchRotation = YRotationMatrix(Pitch);
		CameraRotation = MatrixMultiply(PitchRotation, CameraRotation);
		lookdir = MatxVec(CameraRotation, Target);
		Target = AddVec(Camera, lookdir);
		mat4 CameraMatrix = PointingMatrix(Camera, Target, Up);
		mat4 ViewMatrix = MatrixQuickInverse(CameraMatrix);

		//Draw Mesh
		vector<tri3> TriVector = DrawMesh(obj, Projection, ViewMatrix, World, Camera);

		sort(TriVector.begin(), TriVector.end(), [](tri3& t1, tri3& t2)
			{
				float z1 = (t1.p[0].z + t1.p[1].z + t1.p[2].z) / 3.0f;
				float z2 = (t2.p[0].z + t2.p[1].z + t2.p[2].z) / 3.0f;
				return z1 > z2;
			});


		// 래스터라이징 및 최종 화면 출력.
		RasterRenderMesh(TriVector, ini, mywindow);


		//Draw Watermark
		if (ini.Watermark) {
			FPS = 1;
			FPS /= delta.asSeconds();
			watermark = versionno;
			watermark += " (";
			watermark += to_string(FPS);
			watermark += " FPS)\nX: ";
			watermark += to_string(Camera.x);
			watermark += "ft Y: ";
			watermark += to_string(Camera.y);
			watermark += "ft Z: ";
			watermark += to_string(Camera.z);
			watermark += L"ft\nθY:";
			watermark += to_string(Yaw);
			watermark += L"RAD θP:";
			watermark += to_string(Pitch);
			watermark += L"RAD";

			watermarktext.setString(watermark);
			mywindow.draw(watermarktext);
		}


		//Draw Frame to Window
		mywindow.display();
	}
	return 0;

}


void DrawTriangle(tri3 tri, RenderSettings ini, sf::RenderWindow& mywindow)
{
	sf::VertexArray poly(sf::Triangles, 3);

	poly[0].position = sf::Vector2f(tri.p[0].x, tri.p[0].y);
	poly[1].position = sf::Vector2f(tri.p[1].x, tri.p[1].y);
	poly[2].position = sf::Vector2f(tri.p[2].x, tri.p[2].y);
	poly[0].color = sf::Color(tri.color + 100, tri.color, tri.color);
	poly[1].color = sf::Color(tri.color, tri.color + 100, tri.color);
	poly[2].color = sf::Color(tri.color, tri.color, tri.color + 100);

	mywindow.draw(poly);

	//Draw Triangle's Wireframe if Wireframe is Enabled
	if (ini.Wireframe == true) {
		sf::VertexArray wire(sf::LineStrip, 4);

		wire[0].position = sf::Vector2f(tri.p[0].x, tri.p[0].y);
		wire[1].position = sf::Vector2f(tri.p[1].x, tri.p[1].y);
		wire[2].position = sf::Vector2f(tri.p[2].x, tri.p[2].y);
		wire[3].position = sf::Vector2f(tri.p[0].x, tri.p[0].y);

		wire[0].color = sf::Color(200, 200, 200, 255);
		wire[1].color = sf::Color(200, 200, 200, 255);
		wire[2].color = sf::Color(200, 200, 200, 255);
		wire[3].color = sf::Color(200, 200, 200, 255);

		mywindow.draw(wire);
	}
}

void RasterRenderMesh(vector<tri3> TriVector, RenderSettings ini, sf::RenderWindow& mywindow)
{
	
	for (int i = 0; i < TriVector.size(); ++i)
	{
		tri3 triToRaster = TriVector[i];

		tri3 clipped[2]; // 각 스레드마다 개별적으로 선언되어야함.
		list<tri3> *listTriangles; // 리스트의 배열 제작.

		listTriangles = new list<tri3>[4];
		int nNewTriangles[4];
		for (int i = 0; i < 4; i++)
		{
			listTriangles[i].push_back(triToRaster);
			nNewTriangles[i] = 1;
		}

		#pragma omp parallel num_threads(4)
		for (int p = 0; p < 4; p++)
		{
			int tID = omp_get_thread_num();
			int nTrisToAdd = 0;
			while (nNewTriangles > 0)
			{
				tri3 test = listTriangles[i].front();
				listTriangles[i].pop_front();
				nNewTriangles[i]--;

				switch (tID)
				{
				case 0:	nTrisToAdd = ClipTriangleAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, test, clipped[0], clipped[1]); break;
				case 1:	nTrisToAdd = ClipTriangleAgainstPlane({ 0.0f, float(ini.yres) - 1, 0.0f }, { 0.0f, -1.0f, 0.0f }, test, clipped[0], clipped[1]); break;
				case 2:	nTrisToAdd = ClipTriangleAgainstPlane({ 0.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, test, clipped[0], clipped[1]); break;
				case 3:	nTrisToAdd = ClipTriangleAgainstPlane({ float(ini.xres) - 1, 0.0f, 0.0f }, { -1.0f, 0.0f, 0.0f }, test, clipped[0], clipped[1]); break;
				}

				for (int w = 0; w < nTrisToAdd; w++)
					listTriangles[i].push_back(clipped[w]);
			}
			nNewTriangles[i] = listTriangles[i].size();
		}

		// https://stackoverflow.com/questions/55979935/concatenatee-two-lists-in-c-in-o1-complexity
		list<tri3> finalListTriangles;

		for (int i = 0; i < 4; i++)
		{
			finalListTriangles.splice(finalListTriangles.end(), listTriangles[i]);
		}


		for (auto& Final : finalListTriangles)
		{
			//Draw Triangles to Window
			DrawTriangle(Final, ini, mywindow);
		}
	}

}




vector<tri3> DrawMesh(mesh obj, mat4 Projection, mat4 ViewMatrix, mat4 World, vec3 Camera)
{
	vector<tri3> TriangleVector;
	for (auto tri : obj.polys)
	{
		tri3 Projected, Transformed, Viewed;
		for (int i = 0; i < 3; i++)
			Transformed.p[i] = MatxVec(World, tri.p[i]);

		//Get Normal
		vec3 normalVec3 = NomalizingVector3(Transformed.p[0], Transformed.p[1], Transformed.p[2]);

		vec3 CameraRay = SubVec(Transformed.p[0], Camera);
		vec3 plainZ = { 0.0f, 0.0f, 1.0f };

		if (Dot(normalVec3, CameraRay) < 0.0f)
		{
			//Light Triangle
			vec3 light = { -0.5f, -0.5f, -0.5f };
			light = Norm(light);
			float dotproduct = Dot(light, normalVec3);

			//Move from World Frame to Camera Frame
			for (int i = 0; i < 3; i++)
				Viewed.p[i] = MatxVec(ViewMatrix, Transformed.p[i]);

			// Clip Viewed Triangle against near plane, this could form two additional triangles. 
			int nClippedTriangles = 0;
			tri3 Clip[2];
			nClippedTriangles = ClipTriangleAgainstPlane(plainZ, plainZ, Viewed, Clip[0], Clip[1]);

			vec3 point = { 1, 1, 0 };

			// We may end up with multiple triangles form the clip, so project as required
			for (int i = 0; i < nClippedTriangles; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					//Project Triangles onto 2D
					Projected.p[j] = MatxVec(Projection, Clip[i].p[j]);
					Projected.p[j] = VecdScalar(Projected.p[j], Projected.p[j].w);

					//Unfuck perspective
					Projected.p[j].x *= -1.0f;
					Projected.p[j].y *= -1.0f;

					//Scale Triangles to Screen Size
					Projected.p[j] = AddVec(Projected.p[j], point);
					Projected.p[j].x *= 0.5f * 1920.0f;
					Projected.p[j].y *= 0.5f * 1080.0f;
				}
				Projected.color = 101 - dotproduct * 50;

				//Store Triangles For Sorting
				TriangleVector.push_back(Projected);
			}
		}
	}
	return TriangleVector;
}


vec3 NomalizingVector3(vec3 vector1, vec3 vector2, vec3 vector3)
{
	vec3 subVec1 = SubVec(vector2, vector1);
	vec3 subVec2 = SubVec(vector3, vector1);
	vec3 nomalVec = Cross(subVec1, subVec2);
	return Norm(nomalVec);
}
void CheckAllEvent(sf::RenderWindow* mywindow, RenderSettings* ini, sf::Clock* clock, vec3* Camera, float* Runtime, float* Yaw, float* Pitch, vec3* lookdir, sf::Color sky, sf::Time* delta)
{
	//Check All New Events
	sf::Event event;
	while (mywindow->pollEvent(event))
	{
		//If an event is type "Closed", close the window
		if (event.type == sf::Event::Closed)

			mywindow->close();
		else if (event.type == sf::Event::KeyReleased)
		{
			if (event.key.code == sf::Keyboard::F3)
			{
				if (ini->Watermark)
				{
					ini->Watermark = false;
				}
				else
				{
					ini->Watermark = true;
				}
			}
			else if (event.key.code == sf::Keyboard::F4)
			{
				if (ini->Wireframe)
				{
					ini->Wireframe = false;
				}
				else
				{
					ini->Wireframe = true;
				}
			}
		}

	}
	//Get Time Since Last Frame
	*delta = clock->restart();
	*Runtime = *Runtime + delta->asSeconds();

	// Window의 색깔을 초기화.
	mywindow->clear(sky);

	// 화면 초기 로직 설정.
	vec3 Forward = VecxScalar(*lookdir, 20.0f * delta->asSeconds());
	vec3 Up = { 0,0,1 };
	vec3 Right = Cross(*lookdir, Up);
	vec3 CameraUp = Cross(Right, *lookdir);
	Right = VecxScalar(Right, 20.0f * delta->asSeconds());
	CameraUp = VecxScalar(CameraUp, 20.0f * delta->asSeconds());



	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
	{
		*Camera = AddVec(*Camera, CameraUp);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift))
	{
		*Camera = SubVec(*Camera, CameraUp);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::W))
	{
		*Camera = AddVec(*Camera, Forward);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::A))
	{
		*Camera = SubVec(*Camera, Right);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::S))
	{
		*Camera = SubVec(*Camera, Forward);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::D))
	{
		*Camera = AddVec(*Camera, Right);
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
	{
		*Yaw -= 2.0f * delta->asSeconds();
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
	{
		*Yaw += 2.0f * delta->asSeconds();
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Up))
	{
		*Pitch += 2.0f * delta->asSeconds();
	}
	if (sf::Keyboard::isKeyPressed(sf::Keyboard::Down))
	{
		*Pitch -= 2.0f * delta->asSeconds();
	}
	// 2파이 넘을 경우에 이를 0도로 초기화.
	if (*Yaw >= 2.0f * 3.14159f)
	{
		*Yaw -= 2.0f * 3.14159f;
	}
	if (*Yaw <= 0.0f)
	{
		*Yaw += 2.0f * 3.14159f;
	}
	if (*Pitch >= 3.14159f / 2.0f)
	{
		*Pitch = 3.14100f / 2.0f;
	}
	if (*Pitch <= -3.14159f / 2.0f)
	{
		*Pitch = -3.14100f / 2.0f;
	}
}
