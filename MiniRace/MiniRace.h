#include <blib/App.h>
#include <blib/gl/Vertex.h>
#include <blib/RenderState.h>
#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletDynamics/Vehicle/btRaycastVehicle.h>


namespace blib
{
	class StaticModel;
	class Shader;
}

class DebugDraw;



class MiniRace : public blib::App
{
	blib::StaticModel* track;
	blib::StaticModel* trackCollision;
	blib::StaticModel* car;
	blib::StaticModel* tire;
	blib::Shader* shader;

	enum class Uniforms
	{
		projectionMatrix,
		modelviewMatrix,
		normalMatrix,
		s_texture,
		color,
		texMult,

	};

	btBroadphaseInterface*                  broadphase;
	btDefaultCollisionConfiguration*        collisionConfiguration;
	btCollisionDispatcher*                  dispatcher;
	btSequentialImpulseConstraintSolver*    solver;
	btDiscreteDynamicsWorld*                world;

	DebugDraw* debugDraw;

	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;


	btRigidBody* shipBody;

public:
	MiniRace();
	void init();
	void update(double elapsedTime);
	void draw();
	
};































class DebugDraw : public btIDebugDraw
{
	std::vector<blib::VertexP3C4> vertices;
	blib::RenderState renderstate;
	enum class Uniforms
	{
		ProjectionMatrix,
		CameraMatrix,
	};
	int debugMode;
public:
	DebugDraw(blib::ResourceManager* resourceManager);
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color)	{	}

	void flush(blib::Renderer* renderer, const glm::mat4 &projectionMatrix, const glm::mat4 &cameraMatrix);

	virtual void reportErrorWarning(const char* warningString)		{ printf("%s\n", warningString); }
	virtual void draw3dText(const btVector3& location, const char* textString)		{	}
	virtual void setDebugMode(int debugMode)
	{
		this->debugMode = debugMode;
	}
	virtual int getDebugMode() const
	{
		return debugMode;
	}
};
