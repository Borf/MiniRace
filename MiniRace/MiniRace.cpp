#include "MiniRace.h"

#include <blib/Renderer.h>
#include <blib/SpriteBatch.h>
#include <blib/StaticModel.h>
#include <blib/Shader.h>
#include <blib/RenderState.h>
#include <blib/ResourceManager.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

MiniRace::MiniRace()
{
	appSetup.border = false;
	appSetup.renderer = blib::AppSetup::GlRenderer;
	appSetup.window.setWidth(1920);
	appSetup.window.setHeight(1080);
	appSetup.vsync = true;


}


bool isFrontWheel = true;
float connectionHeight = -0.025f;
float wheelRadius = 0.2f;
float wheelWidth = 0.015f;
btVector3 wheelDirectionCS0(0, -1, 0);
btVector3 wheelAxleCS(1, 0, 0);
btScalar suspensionRestLength(0.2);
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;
float	wheelFriction = 1000;//BT_LARGE_FLOAT;

void MiniRace::init()
{
	track = new blib::StaticModel("assets/models/racetrack-racoon.obj.json", resourceManager, renderer);
	trackCollision = new blib::StaticModel("assets/models/racetrack-racoon-collision.obj.json", resourceManager, renderer);
	tire = new blib::StaticModel("assets/models/tire.fbx.json", resourceManager, renderer);

	car = new blib::StaticModel("assets/models/vehicles/car-parsche-sport-grey.obj.json", resourceManager, renderer);
	shader = resourceManager->getResource<blib::Shader>("default");
	shader->bindAttributeLocation("a_position", 0);
	shader->bindAttributeLocation("a_texcoord", 1);
	shader->bindAttributeLocation("a_normal", 2);
	shader->setUniformName(Uniforms::s_texture, "s_texture", blib::Shader::Int);
	shader->setUniformName(Uniforms::color, "color", blib::Shader::Vec4);
	shader->setUniformName(Uniforms::texMult, "texMult", blib::Shader::Vec4);
	shader->setUniformName(Uniforms::normalMatrix, "normalMatrix", blib::Shader::Mat3);
	shader->setUniformName(Uniforms::modelviewMatrix, "modelviewMatrix", blib::Shader::Mat4);
	shader->setUniformName(Uniforms::projectionMatrix, "projectionMatrix", blib::Shader::Mat4);
	shader->finishUniformSetup();
	shader->setUniform(Uniforms::s_texture, 0);
	shader->setUniform(Uniforms::texMult, glm::vec4(0, 0, 0, 0));


	renderer->renderState.depthTest = true;
	renderer->renderState.activeShader = shader;

	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	world->setGravity(btVector3(0, -9.8f, 0));
//	gContactAddedCallback = CustomMaterialCombinerCallback;
	debugDraw = new DebugDraw(resourceManager);
	debugDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
	world->setDebugDrawer(debugDraw);

	{
		btBoxShape* groundShape = new btBoxShape(btVector3(100, 1, 100));// createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
		btTransform groundTransform;
		groundTransform.setIdentity();
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

		btRigidBody::btRigidBodyConstructionInfo cInfo(0, myMotionState, groundShape, btVector3(0, 0, 0));
		btRigidBody* body = new btRigidBody(cInfo);
		body->setFriction(0.5f);
		world->addRigidBody(body);
		body->setRestitution(0);

		/*
		btTransform tr;
		tr.setOrigin(btVector3(0, 0, 0));//-64.5f,0));
		btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f, 0.5f, 2.f));
		btCompoundShape* compound = new btCompoundShape();
		btTransform localTrans;
		localTrans.setIdentity();
		//localTrans effectively shifts the center of mass with respect to the chassis
		localTrans.setOrigin(btVector3(0, 1, 0));
		compound->addChildShape(localTrans, chassisShape);

		tr.setOrigin(btVector3(0, 0.f, 0));

		shipBody = localCreateRigidBody(800, tr, compound);//chassisShape);
		*/
	}
	{

		const std::vector<blib::math::Triangle3> &triangles = trackCollision->getTriangles();

			btVector3* gVertices = new btVector3[triangles.size()*3];
			int* gIndices = new int[triangles.size()*3];


			for (size_t i = 0; i < triangles.size(); i++)
			{
				gVertices[3 * i + 0] = btVector3(triangles[i].v1.x, triangles[i].v1.y, triangles[i].v1.z);
				gIndices[3 * i + 0] = 3 * i + 0;

				gVertices[3 * i + 1] = btVector3(triangles[i].v2.x, triangles[i].v2.y, triangles[i].v2.z);
				gIndices[3 * i + 1] = 3 * i + 1;

				gVertices[3 * i + 2] = btVector3(triangles[i].v3.x, triangles[i].v3.y, triangles[i].v3.z);
				gIndices[3 * i + 2] = 3 * i + 2;
			}

			//fill
			btTriangleIndexVertexArray* m_indexVertexArrays = new btTriangleIndexVertexArray(triangles.size(),
				gIndices,
				3 * sizeof(int),
				triangles.size()*3, (btScalar*)&gVertices[0].x(), sizeof(btVector3));
			btVector3 aabbMin(-100000, -100000, -100000), aabbMax(100000, 100000, 100000);

			btBvhTriangleMeshShape* levelShape = new btBvhTriangleMeshShape(m_indexVertexArrays, true, aabbMin, aabbMax);
			btTriangleInfoMap* triangleInfoMap = new btTriangleInfoMap();

			btBoxShape* groundShape = new btBoxShape(btVector3(1000, 1, 1000));// createBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(50.)));
			btTransform groundTransform;
			groundTransform.setIdentity();
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);

			btRigidBody::btRigidBodyConstructionInfo cInfo(0, myMotionState, levelShape, btVector3(0, 0, 0));
			btRigidBody* body = new btRigidBody(cInfo);
			body->setFriction(0.5f);
			world->addRigidBody(body);
			body->setRestitution(0);

	}

	{
		{

			btBoxShape* carShape = new btBoxShape(btVector3(0.5, 0.2f, 1));
			btVector3 inertia;
			carShape->calculateLocalInertia(800, inertia);
			btTransform groundTransform;
			groundTransform.setIdentity();
			groundTransform.setOrigin(btVector3(0, 5, 0));
			btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
			btRigidBody::btRigidBodyConstructionInfo cInfo(800, myMotionState, carShape, inertia);
			shipBody = new btRigidBody(cInfo);
			shipBody->forceActivationState(DISABLE_DEACTIVATION);
			world->addRigidBody(shipBody);
		}


		m_vehicleRayCaster = new btDefaultVehicleRaycaster(world);
		m_vehicle = new btRaycastVehicle(m_tuning, shipBody, m_vehicleRayCaster);

		m_vehicle->setCoordinateSystem(0, 1, 2);
		world->addVehicle(m_vehicle);

		btCylinderShapeX* wheelShape = new btCylinderShapeX(btVector3(wheelWidth, wheelRadius, wheelRadius));

		btVector3 connectionPointCS0(0.5f - (0.3*wheelWidth), connectionHeight, -1.8f * 0.5f + wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

		connectionPointCS0 = btVector3(-0.5f + (0.3*wheelWidth), connectionHeight, -1.8f * 0.5f + wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

		isFrontWheel = false;
		connectionPointCS0 = btVector3(-0.5f + (0.3*wheelWidth), connectionHeight, 1.8f * 0.5f - wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

		connectionPointCS0 = btVector3(0.5f - (0.3*wheelWidth), connectionHeight, 1.8f * 0.5f - wheelRadius);
		m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS, suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

		for (int i = 0; i < m_vehicle->getNumWheels(); i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}

		m_vehicle->resetSuspension();
		for (int i = 0; i < m_vehicle->getNumWheels(); i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i, true);
			m_vehicle->setBrake(0, i);
		}

	}



}

float rot = 0;
float power = 0;

void MiniRace::update(double elapsedTime)
{
	if (keyState.isPressed(blib::Key::ESC))
		running = false;

	if (keyState.isPressed(blib::Key::LEFT))
		rot += elapsedTime * 45.0f;
	else if (keyState.isPressed(blib::Key::RIGHT))
		rot -= elapsedTime * 45.0f;

	if (keyState.isPressed(blib::Key::W))
	{
		power += elapsedTime*500.0f;
		if (power > 1000)
			power = 1000;
	}
	else
		power = 0;


	if (keyState.isPressed(blib::Key::A))
	{
		m_vehicle->setSteeringValue(0.3, 0);
		m_vehicle->setSteeringValue(0.3, 1);
	}
	else if (keyState.isPressed(blib::Key::D))
	{
		m_vehicle->setSteeringValue(-0.3, 0);
		m_vehicle->setSteeringValue(-0.3, 1);
	}
	else
	{
		m_vehicle->setSteeringValue(0, 0);
		m_vehicle->setSteeringValue(0, 1);
	}

	m_vehicle->applyEngineForce(power, 0);
	m_vehicle->applyEngineForce(power, 1);
	m_vehicle->applyEngineForce(power, 2);
	m_vehicle->applyEngineForce(power, 3);

	world->stepSimulation((float)elapsedTime, 100);
	


}

void MiniRace::draw()
{
	renderer->clear(glm::vec4(1, 1, 0, 1), blib::Renderer::Color | blib::Renderer::Depth);

	btTransform tr;

	btScalar m[16];
	shipBody->getMotionState()->getWorldTransform(tr);
	tr.getOpenGLMatrix(m);

	glm::mat4 projectionMatrix = glm::perspective(80.0f, 1.0f, 0.1f, 200.0f);
	glm::mat4 viewMatrix;
	viewMatrix = glm::translate(viewMatrix, glm::vec3(0, 0, -7));
	viewMatrix = glm::rotate(viewMatrix, 30.0f, glm::vec3(1, 0, 0));
	viewMatrix = glm::rotate(viewMatrix, rot, glm::vec3(0, 1, 0));
	viewMatrix *= glm::inverse(glm::make_mat4(m));
	glm::mat4 modelMatrix = glm::mat4();

	shader->setUniform(Uniforms::projectionMatrix, projectionMatrix );
	shader->setUniform(Uniforms::color, glm::vec4(0, 0, 0, 0));
	shader->setUniform(Uniforms::texMult, glm::vec4(1,1,1,1));


	shader->setUniform(Uniforms::modelviewMatrix, viewMatrix * modelMatrix);
	shader->setUniform(Uniforms::normalMatrix, glm::transpose(glm::inverse(glm::mat3(modelMatrix))));
	track->draw(renderer->renderState, renderer, [this](const blib::Material& material)
	{
		renderer->renderState.activeTexture[0] = material.texture;
	});


	shipBody->getMotionState()->getWorldTransform(tr);
	tr.getOpenGLMatrix(m);
	modelMatrix = glm::make_mat4(m);
	modelMatrix = glm::translate(modelMatrix, glm::vec3(0, -0.25f, 0));
	shader->setUniform(Uniforms::modelviewMatrix, viewMatrix * modelMatrix);
	shader->setUniform(Uniforms::normalMatrix, glm::transpose(glm::inverse(glm::mat3(modelMatrix))));
	car->draw(renderer->renderState, renderer, [this](const blib::Material& material)
	{
		renderer->renderState.activeTexture[0] = material.texture;
	});

	//world->debugDrawWorld();
	//debugDraw->flush(renderer, projectionMatrix, viewMatrix);


	for (int i = 0; i < m_vehicle->getNumWheels(); i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i, true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
	//	m_shapeDrawer->drawOpenGL(m, m_wheelShape, wheelColor, getDebugMode(), worldBoundsMin, worldBoundsMax);

		btWheelInfo& info = m_vehicle->getWheelInfo(i);

		modelMatrix = glm::mat4();
		modelMatrix = glm::translate(modelMatrix, glm::vec3(0, 0, 0));
		modelMatrix *= glm::make_mat4(m);
		modelMatrix = glm::scale(modelMatrix, glm::vec3(wheelWidth, -wheelRadius, wheelRadius));
		shader->setUniform(Uniforms::modelviewMatrix, viewMatrix * modelMatrix);
		shader->setUniform(Uniforms::normalMatrix, glm::transpose(glm::inverse(glm::mat3(modelMatrix))));
		tire->draw(renderer->renderState, renderer, [this](const blib::Material& material)
		{
			renderer->renderState.activeTexture[0] = material.texture;
		});


	}

}























DebugDraw::DebugDraw(blib::ResourceManager* resourceManager)
{
	renderstate.activeShader = resourceManager->getResource<blib::Shader>("bulletdebug");
	renderstate.activeShader->setUniformName(Uniforms::ProjectionMatrix, "projectionMatrix", blib::Shader::Mat4);
	renderstate.activeShader->setUniformName(Uniforms::CameraMatrix, "cameraMatrix", blib::Shader::Mat4);
	renderstate.activeShader->finishUniformSetup();
	renderstate.depthTest = false;
}


void DebugDraw::drawLine(const btVector3& from, const btVector3& to, const btVector3& color)
{
	if (vertices.size() > 10000000)
		return;
	glm::vec4 c(color.m_floats[0], color.m_floats[1], color.m_floats[2], 1.0f);
	vertices.push_back(blib::VertexP3C4(glm::vec3(from.m_floats[0], from.m_floats[1], from.m_floats[2]), c));
	vertices.push_back(blib::VertexP3C4(glm::vec3(to.m_floats[0], to.m_floats[1], to.m_floats[2]), c));
}

void DebugDraw::flush(blib::Renderer* renderer, const glm::mat4 &projectionMatrix, const glm::mat4 &cameraMatrix)
{
	if (!vertices.empty())
	{
		renderstate.activeShader->setUniform(Uniforms::ProjectionMatrix, projectionMatrix);
		renderstate.activeShader->setUniform(Uniforms::CameraMatrix, cameraMatrix);
		renderer->drawLines(vertices, renderstate);
		vertices.clear();
	}
}
