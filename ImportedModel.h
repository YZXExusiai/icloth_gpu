#pragma once
#include <vector>

class ImportedModel
{
private:
	int numVertices;
	int numNode;
	std::vector<glm::vec3> vertices;
	std::vector<glm::vec3> node;
	std::vector<int> FaceId;
	std::vector<glm::vec2> texCoords;
	std::vector<glm::vec3> normalVecs;
public:
	ImportedModel();
	ImportedModel(const char* filePath);
	int getNumVertices();
	int getNodeNum();
	std::vector<glm::vec3> getVertices();//��ȡ�����Ѿ���������һ�鹹����Ķ�������
	std::vector<glm::vec3> getNode();//��ȡ���Ƕ�������
	std::vector<glm::vec2> getTextureCoords();
	std::vector<int> getFaceId();
	//std::vector<glm::vec3> getNormals();
};

class ModelImporter
{
private:
	std::vector<float> vertVals;
	std::vector<float> triangleVerts;
	std::vector<float> textureCoords;
	std::vector<float> stVals;
	std::vector<float> normals;
	std::vector<float> normVals;
	std::vector<int> FaceId;
public:
	ModelImporter();
	void parseOBJ(const char* filePath);
	int getNumVertices();
	std::vector<float> getNode();
	std::vector<float> getVertices();
	std::vector<float> getTextureCoordinates();
	std::vector<float> getNormals();
	std::vector<int> getFaceId();
};