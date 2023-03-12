#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "BVH.h"
#include "BVH8Converter.h"

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "crc64.h"

#include "GLUtils.h"


const double PI = 3.14159265359;
const double DEG2RAD = PI / 180.0;

struct CameraConst
{
	glm::mat4 ProjMat;
	glm::mat4 ViewMat;
	glm::mat4 InvProjMat;
	glm::mat4 InvViewMat;
	glm::vec4 EyePos;
};


struct ModelConst
{
	glm::mat4 ModelMat;
	glm::mat4 NormalMat;
};

typedef std::unique_ptr<GLBuffer> Attribute;
typedef std::unique_ptr<GLBuffer> Index;

struct Primitive
{
	glm::mat4 model_matrix;

	int num_pos = 0;
	int num_face = 0;

	std::vector<glm::vec4> pos;
	std::vector<glm::vec4> norm;
	std::vector<glm::ivec3> faces;

	GLDynBuffer m_constant_model;

	Attribute pos_buf;
	Attribute normal_buf;

	Index index_buf;

	void update_constant()
	{
		ModelConst c;
		c.ModelMat = model_matrix;
		c.NormalMat = glm::transpose(glm::inverse(model_matrix));
		m_constant_model.upload(&c);
	}

	Primitive() : m_constant_model(sizeof(ModelConst), GL_UNIFORM_BUFFER)
	{
		model_matrix = glm::identity<glm::mat4>();
	}	

	void create()
	{
		pos_buf = (std::unique_ptr<GLBuffer>)(new GLBuffer(sizeof(glm::vec4) * num_pos, GL_SHADER_STORAGE_BUFFER));
		pos_buf->upload(pos.data());

		normal_buf = (std::unique_ptr<GLBuffer>)(new GLBuffer(sizeof(glm::vec4) * num_pos, GL_SHADER_STORAGE_BUFFER));
		normal_buf->upload(norm.data());

		index_buf = (std::unique_ptr<GLBuffer>)(new GLBuffer(sizeof(glm::ivec3) * num_face, GL_SHADER_STORAGE_BUFFER));
		index_buf->upload(faces.data());
	}

	void CreateBox(float width, float height, float depth)
	{
		pos.clear();
		norm.clear();
		faces.clear();

		float half_w = width * 0.5f;
		float half_h = height * 0.5f;
		float half_d = depth * 0.5f;

		// x positive
		{
			int v_start = (int)pos.size();
			pos.push_back({ half_w, half_h, half_d, 1.0f });
			pos.push_back({ half_w, half_h, -half_d, 1.0f });
			pos.push_back({ half_w, -half_h, half_d, 1.0f });
			pos.push_back({ half_w, -half_h, -half_d, 1.0f });

			norm.push_back({ 1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ 1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ 1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ 1.0f, 0.0f, 0.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		// x negative
		{
			int v_start = (int)pos.size();
			pos.push_back({ -half_w, half_h, -half_d, 1.0f });
			pos.push_back({ -half_w, half_h, half_d, 1.0f });
			pos.push_back({ -half_w, -half_h, -half_d, 1.0f });
			pos.push_back({ -half_w, -half_h, half_d, 1.0f });

			norm.push_back({ -1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ -1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ -1.0f, 0.0f, 0.0f, 0.0f });
			norm.push_back({ -1.0f, 0.0f, 0.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		// y positive
		{
			int v_start = (int)pos.size();
			pos.push_back({ -half_w, half_h, -half_d, 1.0f });
			pos.push_back({ half_w, half_h, -half_d, 1.0f });
			pos.push_back({ -half_w, half_h, half_d, 1.0f });
			pos.push_back({ half_w, half_h, half_d, 1.0f });

			norm.push_back({ 0.0f, 1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, 1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, 1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, 1.0f, 0.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		// y negative
		{
			int v_start = (int)pos.size();
			pos.push_back({ -half_w, -half_h, half_d, 1.0f });
			pos.push_back({ half_w, -half_h, half_d, 1.0f });
			pos.push_back({ -half_w, -half_h, -half_d, 1.0f });
			pos.push_back({ half_w, -half_h, -half_d, 1.0f });

			norm.push_back({ 0.0f, -1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, -1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, -1.0f, 0.0f, 0.0f });
			norm.push_back({ 0.0f, -1.0f, 0.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		// z positive
		{
			int v_start = (int)pos.size();
			pos.push_back({ -half_w, half_h, half_d, 1.0f });
			pos.push_back({ half_w, half_h, half_d, 1.0f });
			pos.push_back({ -half_w, -half_h, half_d, 1.0f });
			pos.push_back({ half_w, -half_h, half_d, 1.0f });

			norm.push_back({ 0.0f, 0.0f, 1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, 1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, 1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, 1.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		// z negative
		{
			int v_start = (int)pos.size();
			pos.push_back({ half_w, half_h, -half_d, 1.0f });
			pos.push_back({ -half_w, half_h, -half_d, 1.0f });
			pos.push_back({ half_w, -half_h, -half_d, 1.0f });
			pos.push_back({ -half_w, -half_h, -half_d, 1.0f });

			norm.push_back({ 0.0f, 0.0f, -1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, -1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, -1.0f, 0.0f });
			norm.push_back({ 0.0f, 0.0f, -1.0f, 0.0f });

			faces.push_back({ v_start + 2, v_start + 1, v_start });
			faces.push_back({ v_start + 1, v_start + 2, v_start + 3 });
		}

		num_pos = (int)pos.size();
		num_face = (int)faces.size();
		create();

	}

	void LoadObj(const char* filename)
	{
		pos.clear();
		norm.clear();
		faces.clear();

		tinyobj::attrib_t                attrib;
		std::vector<tinyobj::shape_t>    shapes;
		std::vector<tinyobj::material_t> materials;
		std::string                      err;

		tinyobj::LoadObj(&attrib, &shapes, &materials, &err, filename);

		std::unordered_map<uint64_t, size_t> ind_map;

		int num_meshes = (int)shapes.size();
		for (int i = 0; i < num_meshes; i++)
		{
			tinyobj::shape_t& shape = shapes[i];
			int face_start = (int)faces.size();
			faces.resize(face_start + shape.mesh.indices.size() / 3);
			for (int j = 0; j < (int)shape.mesh.indices.size(); j++)
			{
				tinyobj::index_t& index = shape.mesh.indices[j];
				struct Attributes
				{
					glm::vec3 pos = { 0.0f, 0.0f, 0.0f };
					glm::vec3 norm = { 0.0f, 0.0f, 0.0f };
				};

				Attributes att;

				float* vp = &attrib.vertices[3 * index.vertex_index];
				att.pos = glm::vec3(vp[0], vp[1], vp[2]);

				float* np = &attrib.normals[3 * index.normal_index];
				att.norm = glm::vec3(np[0], np[1], np[2]);

				int idx;
				uint64_t hash = crc64(0, (unsigned char*)&att, sizeof(Attributes));

				auto iter = ind_map.find(hash);
				if (iter == ind_map.end())
				{
					idx = (int)pos.size();
					pos.push_back(glm::vec4(att.pos, 1.0f));
					if (attrib.normals.size() > 0)
					{
						norm.push_back(glm::vec4(att.norm, 0.0f));
					}
					ind_map[hash] = idx;
				}
				else
				{
					idx = ind_map[hash];
				}
				faces[face_start + j / 3][j % 3] = idx;
			}

		}

		num_pos = (int)pos.size();
		num_face = (int)faces.size();
		create();
	}
};

class BLAS
{
public:
	BLAS(const Primitive* primitive);
	~BLAS();

	std::vector<flex_bvh::Triangle> m_triangles;
	flex_bvh::BVH2 m_bvh2;
	flex_bvh::BVH8 m_bvh8;

	std::unique_ptr<GLBuffer> m_buf_bvh8;
	std::unique_ptr<GLBuffer> m_buf_triangles;
	std::unique_ptr<GLBuffer> m_buf_indices;
};

BLAS::BLAS(const Primitive* primitive)
{
	const glm::mat4& model_matrix = primitive->model_matrix;
	for (int i = 0; i < primitive->num_face; i++)
	{
		glm::ivec3 face = primitive->faces[i];
		glm::vec4 v0, v1, v2;
		v0 = primitive->pos[face.x];
		v1 = primitive->pos[face.y];
		v2 = primitive->pos[face.z];

		v0 = model_matrix * v0;
		v1 = model_matrix * v1;
		v2 = model_matrix * v2;

		m_triangles.emplace_back(flex_bvh::Triangle(v0, v1, v2));
	}

	m_bvh2.create_from_triangles(m_triangles);
	flex_bvh::ConvertBVH2ToBVH8(m_bvh2, m_bvh8);

	m_buf_bvh8 = std::unique_ptr<GLBuffer>(new GLBuffer(sizeof(flex_bvh::BVHNode8) * m_bvh8.nodes.size(), GL_SHADER_STORAGE_BUFFER));
	m_buf_bvh8->upload(m_bvh8.nodes.data());

	std::vector<glm::vec4> triangles(m_bvh8.indices.size()*3);
	for (size_t i = 0; i < m_bvh8.indices.size(); i++)
	{
		int index = m_bvh8.indices[i];
		const flex_bvh::Triangle& tri = m_triangles[index];	
		triangles[i * 3] = glm::vec4(tri.position_0, 1.0f);
		triangles[i * 3 + 1] = glm::vec4(tri.position_1 - tri.position_0, 0.0f);
		triangles[i * 3 + 2] = glm::vec4(tri.position_2 - tri.position_0, 0.0f);
	}

	m_buf_triangles = std::unique_ptr<GLBuffer>(new GLBuffer(sizeof(glm::vec4) * triangles.size(), GL_SHADER_STORAGE_BUFFER));
	m_buf_triangles->upload(triangles.data());

	m_buf_indices = std::unique_ptr<GLBuffer>(new GLBuffer(sizeof(int) * m_bvh8.indices.size(), GL_SHADER_STORAGE_BUFFER));
	m_buf_indices->upload(m_bvh8.indices.data());

}

BLAS::~BLAS()
{

}

void test1()
{
	int width = 1280;
	int height = 720;
	float aspect = (float)width / (float)height;
	glm::mat4 projection = glm::perspective(45.0f * (float)DEG2RAD, aspect, 0.1f, 100.0f);
	glm::mat4 view = glm::lookAt(glm::vec3(1.0f, 2.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0, 1.0, 0.0));

	Primitive prim;
	prim.LoadObj("shaderBall.obj");
	prim.model_matrix = glm::translate(prim.model_matrix, glm::vec3(0.0f, -1.0f, 0.0f));
	prim.model_matrix = glm::rotate(prim.model_matrix, -(float)PI * 0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
	prim.model_matrix = glm::rotate(prim.model_matrix, -(float)PI * 0.5f, glm::vec3(1.0f, 0.0f, 0.0f));

	glm::mat4 normalMat = glm::transpose(glm::inverse(prim.model_matrix));
	glm::mat4 invProjection = glm::inverse(projection);
	glm::mat4 invView = glm::inverse(view);
	glm::vec3 eyePos = glm::vec3(invView[3]);
	glm::ivec2 viewport_size(width, height);

	BLAS blas(&prim);

	std::vector<uint8_t> test(width * height);
	std::vector<uint8_t> depth(width * height);
	std::vector<glm::u8vec3> normal(width * height);

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			glm::ivec2 screen(x, height - 1 - y);
			glm::vec4 clip((glm::vec2(screen) + 0.5f) / glm::vec2(viewport_size) * 2.0f - 1.0f, 0.0f, 1.0f);
			glm::vec4 view = invProjection * clip;
			view /= view.w;
			glm::vec3 world = glm::vec3(invView * view);
			glm::vec3 dir = glm::normalize(world - eyePos);

			flex_bvh::Ray ray = { eyePos, dir };
			flex_bvh::Intersection intersection;
			blas.m_bvh2.intersect(ray, intersection, blas.m_triangles);

			float d = 1.0f;
			glm::vec3 norm = glm::vec3(0.0f, 0.0f, 1.0f);
			uint8_t v_test = 0;

			if (intersection.triangle_index >= 0)
			{
				v_test = 255;

				d = (intersection.t - 2.0f) / 5.0f;
				if (d < 0.0f) d = 0.0f;
				if (d > 1.0f) d = 1.0f;

				int face_id = intersection.triangle_index;
				float u = intersection.u;
				float v = intersection.v;
				glm::ivec3 face = prim.faces[face_id];
				glm::vec4 norm0 = prim.norm[face.x];
				glm::vec4 norm1 = prim.norm[face.y];
				glm::vec4 norm2 = prim.norm[face.z];
				glm::vec4 normal = (1.0f - u - v) * norm0 + u * norm1 + v * norm2;

				norm = glm::vec3(normalMat * normal);

			}
			test[x + y * width] = v_test;
			depth[x + y * width] = (uint8_t)(d * 255.0f + 0.5f);
			normal[x + y * width] = glm::u8vec3((norm * 0.5f + 0.5f) * 255.0f + 0.5f);

		}
	}


	{
		FILE* fp = fopen("dmp_test.raw", "wb");
		fwrite(test.data(), 1, width * height, fp);
		fclose(fp);
	}

	{
		FILE* fp = fopen("dmp_depth.raw", "wb");
		fwrite(depth.data(), 1, width * height, fp);
		fclose(fp);
	}

	{
		FILE* fp = fopen("dmp_normal.raw", "wb");
		fwrite(normal.data(), 3, width * height, fp);
		fclose(fp);
	}

}

static std::string g_compute =
R"(#version 430

uint ray_get_octant_inv4(in vec3 ray_direction)
{
	return (ray_direction.x < 0.0 ? 0 : 0x04040404) |
		(ray_direction.y < 0.0 ? 0 : 0x02020202) |
		(ray_direction.z < 0.0 ? 0 : 0x01010101);
}

struct BVH8Node
{
	vec4 node_0;
	vec4 node_1;
	vec4 node_2;
	vec4 node_3;
	vec4 node_4; 
};

layout (std430, binding = 0) buffer BVH8
{
	BVH8Node bvh8_nodes[];
};

layout (std430, binding = 1) buffer Triangles
{
	vec4 triangles[];
};

layout (std430, binding = 2) buffer Indices
{
	int indices[];
};

struct Ray
{
	vec3 origin;
	float tmin;
	vec3 direction;	
	float tmax;
};

struct Intersection
{
	int triangle_index;
	float t;
	float u;
	float v;
};

uint extract_byte(uint x, uint i) 
{
	return (x >> (i * 8)) & 0xff;
}

uint sign_extend_s8x4(uint x) 
{
	return ((x >> 7) & 0x01010101) * 0xff;
}

uint bvh8_node_intersect(in Ray ray, uint oct_inv4, float max_distance, in BVH8Node node)
{
	vec3 p = node.node_0.xyz;
	
	uint e_imask = floatBitsToUint(node.node_0.w);
	uint e_x = extract_byte(e_imask, 0);
	uint e_y = extract_byte(e_imask, 1);
	uint e_z = extract_byte(e_imask, 2);

	vec3 adjusted_ray_direction_inv = vec3(
		uintBitsToFloat(e_x << 23) / ray.direction.x,
		uintBitsToFloat(e_y << 23) / ray.direction.y,
		uintBitsToFloat(e_z << 23) / ray.direction.z
	);

	vec3 adjusted_ray_origin = (p - ray.origin) / ray.direction;

	uint hit_mask = 0;

	for (int i = 0; i < 2; i++) 
	{
		uint meta4 = floatBitsToUint(i == 0 ? node.node_1.z : node.node_1.w);
		
		uint is_inner4   = (meta4 & (meta4 << 1)) & 0x10101010;
		uint inner_mask4 = sign_extend_s8x4(is_inner4 << 3);
		uint bit_index4  = (meta4 ^ (oct_inv4 & inner_mask4)) & 0x1f1f1f1f;
		uint child_bits4 = (meta4 >> 5) & 0x07070707;

		// Select near and far planes based on ray octant
		uint q_lo_x = floatBitsToUint(i == 0 ? node.node_2.x : node.node_2.y);
		uint q_hi_x = floatBitsToUint(i == 0 ? node.node_2.z : node.node_2.w);

		uint q_lo_y = floatBitsToUint(i == 0 ? node.node_3.x : node.node_3.y);
		uint q_hi_y = floatBitsToUint(i == 0 ? node.node_3.z : node.node_3.w);

		uint q_lo_z = floatBitsToUint(i == 0 ? node.node_4.x : node.node_4.y);
		uint q_hi_z = floatBitsToUint(i == 0 ? node.node_4.z : node.node_4.w);

		uint x_min = ray.direction.x < 0.0 ? q_hi_x : q_lo_x;
		uint x_max = ray.direction.x < 0.0 ? q_lo_x : q_hi_x;

		uint y_min = ray.direction.y < 0.0 ? q_hi_y : q_lo_y;
		uint y_max = ray.direction.y < 0.0 ? q_lo_y : q_hi_y;

		uint z_min = ray.direction.z < 0.0 ? q_hi_z : q_lo_z;
		uint z_max = ray.direction.z < 0.0 ? q_lo_z : q_hi_z;

		for (int j = 0; j < 4; j++) 
		{
			// Extract j-th byte
			vec3 tmin3 = vec3(float(extract_byte(x_min, j)), float(extract_byte(y_min, j)), float(extract_byte(z_min, j)));
			vec3 tmax3 = vec3(float(extract_byte(x_max, j)), float(extract_byte(y_max, j)), float(extract_byte(z_max, j)));

			// Account for grid origin and scale
			tmin3 = tmin3 * adjusted_ray_direction_inv + adjusted_ray_origin;
			tmax3 = tmax3 * adjusted_ray_direction_inv + adjusted_ray_origin;

			float tmin = max(max(tmin3.x, tmin3.y), max(tmin3.z, ray.tmin));
			float tmax = min(min(tmax3.x, tmax3.y), min(tmax3.z, max_distance));

			bool intersected = tmin < tmax;
			if (intersected) 
			{
				uint child_bits = extract_byte(child_bits4, j);
				uint bit_index  = extract_byte(bit_index4,  j);
				hit_mask |= child_bits << bit_index;
			}
		}
	}

	return hit_mask;
}

bool triangle_intersect(int triangle_id, in Ray ray, float max_t, out float t, out float u, out float v, int culling)
{
	vec3 pos0 = triangles[triangle_id*3].xyz;
	vec3 edge1 = triangles[triangle_id*3 + 1].xyz;
	vec3 edge2 = triangles[triangle_id*3 + 2].xyz;
	
	vec3 h = cross(ray.direction, edge2);
	float a = dot(edge1, h);

	if ((culling == 1 && a<=0.0) || ((culling == 2) && a>=0.0) || a==0.0) return false;
	
	float f = 1.0 / a;
	vec3 s = ray.origin - pos0;
	u = f * dot(s, h);

	if (u < 0.0 || u > 1.0) return false;
	
	vec3 q = cross(s, edge1);
	v = f * dot(ray.direction, q);

	if (v < 0.0 || (u + v)> 1.0) return false;
	t = f * dot(edge2, q);

	if (t < ray.tmin) return false;	
	return t <= max_t;
}

#define BVH_STACK_SIZE 32
#define SHARED_STACK_SIZE 8
#define LOCAL_STACK_SIZE (BVH_STACK_SIZE - SHARED_STACK_SIZE)
shared uvec2 shared_stack_bvh8[SHARED_STACK_SIZE*64];

#define SHARED_STACK_INDEX(offset) ((gl_LocalInvocationID.y * SHARED_STACK_SIZE + offset) * 32 + gl_LocalInvocationID.x)

void stack_push(inout uvec2 stack[LOCAL_STACK_SIZE], inout int stack_size, in uvec2 item) {	

	if (stack_size < SHARED_STACK_SIZE) 
	{
		shared_stack_bvh8[SHARED_STACK_INDEX(stack_size)] = item;
	} 
	else 
	{
		stack[stack_size - SHARED_STACK_SIZE] = item;
	}
	stack_size++;
}

uvec2 stack_pop(in uvec2 stack[LOCAL_STACK_SIZE], inout int stack_size) 
{
	stack_size--;
	if (stack_size < SHARED_STACK_SIZE) 
	{
		return shared_stack_bvh8[SHARED_STACK_INDEX(stack_size)];
	} 
	else 
	{
		return stack[stack_size - SHARED_STACK_SIZE];
	}
}


void intersect(in Ray ray, out Intersection ray_hit, int culling)
{
	ray_hit.triangle_index = -1;
	ray_hit.t =  ray.tmax < 0.0 ? 3.402823466e+38 : ray.tmax;
	ray_hit.u = 0.0;
	ray_hit.v = 0.0;
	
	uvec2 stack[LOCAL_STACK_SIZE]; 
	int stack_size = 0;

	uint oct_inv4 = ray_get_octant_inv4(ray.direction);
	uvec2 current_group = uvec2(0, 0x80000000);
	
	while (stack_size > 0 || current_group.y!=0)
	{
		uvec2 triangle_group;
		if ((current_group.y & 0xff000000)!=0)
		{
			uint hits_imask = current_group.y;
			int child_index_offset = findMSB(hits_imask);
			uint child_index_base   = current_group.x;

			// Remove n from current_group;
			current_group.y &= ~(1 << child_index_offset);

			// If the node group is not yet empty, push it on the stack
			if ((current_group.y & 0xff000000)!=0) 
			{
				stack_push(stack, stack_size, current_group);
			}

			uint slot_index     = (child_index_offset - 24) ^ (oct_inv4 & 0xff);
			uint relative_index = bitCount(hits_imask & ~(0xffffffff << slot_index));

			uint child_node_index = child_index_base + relative_index;
			BVH8Node node = bvh8_nodes[child_node_index];
			uint hitmask = bvh8_node_intersect(ray, oct_inv4, ray_hit.t, node);

			uint imask = extract_byte(floatBitsToUint(node.node_0.w), 3);			

			current_group.x = floatBitsToUint(node.node_1.x); // Child    base offset
			triangle_group.x = floatBitsToUint(node.node_1.y); // Triangle base offset

			current_group.y = (hitmask & 0xff000000) | imask;
			triangle_group.y = (hitmask & 0x00ffffff);
		}
		else 
		{
			triangle_group = current_group;
			current_group  = uvec2(0);
		}

		while (triangle_group.y != 0)
		{
			int triangle_index = findMSB(triangle_group.y);
			triangle_group.y &= ~(1 << triangle_index);

			int tri_idx = int(triangle_group.x + triangle_index);
			float t,u,v;
			if (triangle_intersect(tri_idx, ray, ray_hit.t, t, u, v, culling))
			{
				ray_hit.triangle_index = indices[tri_idx];				
				ray_hit.t = t;
				ray_hit.u = u;
				ray_hit.v = v;
			}
		}			

		if ((current_group.y & 0xff000000) == 0) 
		{
			if (stack_size == 0) break;
			current_group = stack_pop(stack, stack_size);			
		}
	}
}


layout (std140, binding = 3) uniform Camera
{
	mat4 uProjMat;
	mat4 uViewMat;	
	mat4 uInvProjMat;
	mat4 uInvViewMat;	
	vec3 uEyePos;
};


layout (std140, binding = 4) uniform Model
{
	mat4 uModelMat;
	mat4 uNormalMat;
};


layout (std430, binding = 5) buffer Faces
{
	int faces[];
};

layout (std430, binding = 6) buffer Normals
{
	vec4 normals[];
};

layout (binding=0, r8) uniform image2D uOut_test;
layout (binding=1, r8) uniform image2D uOut_depth;
layout (binding=2, rgba8) uniform image2D uOut_normal;

layout(local_size_x = 32, local_size_y = 2) in;

void main()
{
	ivec2 size = imageSize(uOut_test);
	ivec2 id = ivec3(gl_GlobalInvocationID).xy;	
	if (id.x>= size.x || id.y >=size.y) return;

	ivec2 screen = ivec2(id.x, size.y - 1 - id.y);
	vec4 clip= vec4((vec2(screen) + 0.5)/vec2(size)*2.0-1.0, 0.0, 1.0);
	vec4 view = uInvProjMat * clip;
	view /= view.w;
	vec3 world = vec3(uInvViewMat*view);
	vec3 dir = normalize(world - uEyePos);

	Ray ray;
	ray.origin = uEyePos;
	ray.direction = dir;
	ray.tmin = 0.005;
	ray.tmax = -1.0;

	Intersection ray_hit;
	intersect(ray, ray_hit, 1);

	float d = 1.0;
	vec3 norm = vec3(0.0f, 0.0f, 1.0f);

	float test = 0.0;
	if (ray_hit.triangle_index >= 0)
	{
		test = 1.0;
		d = clamp((ray_hit.t - 2.0f) / 5.0, 0.0, 1.0);

		int face_id = ray_hit.triangle_index;
		float u = ray_hit.u;
		float v = ray_hit.v;
		int vert_idx0 = faces[face_id*3];
		int vert_idx1 = faces[face_id*3 + 1];
		int vert_idx2 = faces[face_id*3 + 2];
		vec3 norm0 = normals[vert_idx0].xyz;
		vec3 norm1 = normals[vert_idx1].xyz;
		vec3 norm2 = normals[vert_idx2].xyz;
		vec3 normal =  (1.0 - u - v) * norm0 + u * norm1 + v * norm2;
		
		norm = vec3(uNormalMat * vec4(normal, 0.0));
	}
	imageStore(uOut_test, id, vec4(test));	
	imageStore(uOut_depth, id, vec4(d));	
	imageStore(uOut_normal, id, vec4(norm*0.5 + 0.5, 0.0));	
}

)";

class RGBATexture
{
public:
	RGBATexture()
	{
		m_tex = std::unique_ptr<GLTexture2D>(new GLTexture2D);
	}

	bool update(int width, int height)
	{
		if (m_width != width || m_height != height)
		{
			glBindTexture(GL_TEXTURE_2D, m_tex->tex_id);
			glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, width, height);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			glBindTexture(GL_TEXTURE_2D, 0);

			m_width = width;
			m_height = height;

			return true;
		}
		return false;
	}


	int m_width = -1;
	int m_height = -1;

	std::unique_ptr<GLTexture2D> m_tex;
};

class R8Texture
{
public:
	R8Texture()
	{
		m_tex = std::unique_ptr<GLTexture2D>(new GLTexture2D);
	}

	bool update(int width, int height)
	{
		if (m_width != width || m_height != height)
		{
			glBindTexture(GL_TEXTURE_2D, m_tex->tex_id);
			glTexStorage2D(GL_TEXTURE_2D, 1, GL_R8, width, height);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
			glBindTexture(GL_TEXTURE_2D, 0);

			m_width = width;
			m_height = height;

			return true;
		}
		return false;
	}


	int m_width = -1;
	int m_height = -1;

	std::unique_ptr<GLTexture2D> m_tex;
};


void test2()
{
	int width = 1280;
	int height = 720;
	float aspect = (float)width / (float)height;
	glm::mat4 projection = glm::perspective(45.0f * (float)DEG2RAD, aspect, 0.1f, 100.0f);
	glm::mat4 view = glm::lookAt(glm::vec3(1.0f, 2.0f, 5.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0, 1.0, 0.0));

	Primitive prim;
	prim.LoadObj("shaderBall.obj");
	prim.model_matrix = glm::translate(prim.model_matrix, glm::vec3(0.0f, -1.0f, 0.0f));
	prim.model_matrix = glm::rotate(prim.model_matrix, -(float)PI * 0.5f, glm::vec3(0.0f, 1.0f, 0.0f));
	prim.model_matrix = glm::rotate(prim.model_matrix, -(float)PI * 0.5f, glm::vec3(1.0f, 0.0f, 0.0f));
	prim.update_constant();
	
	glm::mat4 normalMat = glm::transpose(glm::inverse(prim.model_matrix));
	glm::mat4 invProjection = glm::inverse(projection);
	glm::mat4 invView = glm::inverse(view);
	glm::vec3 eyePos = glm::vec3(invView[3]);
	glm::ivec2 viewport_size(width, height);

	BLAS blas(&prim);
	
	GLDynBuffer constant_camera(sizeof(CameraConst), GL_UNIFORM_BUFFER);

	CameraConst c;
	c.ProjMat = projection;
	c.ViewMat = view;
	c.InvProjMat = invProjection;
	c.InvViewMat = invView;
	c.EyePos = glm::vec4(eyePos, 1.0f);
	constant_camera.upload(&c);

	R8Texture tex_test;
	tex_test.update(width, height);

	R8Texture tex_depth;
	tex_depth.update(width, height);

	RGBATexture tex_normal;
	tex_normal.update(width, height);

	std::unique_ptr<GLProgram> prog;
	{
		GLShader comp_shader(GL_COMPUTE_SHADER, g_compute.c_str());
		prog = std::unique_ptr<GLProgram>(new GLProgram(comp_shader));
	}

	glUseProgram(prog->m_id);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, blas.m_buf_bvh8->m_id);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, blas.m_buf_triangles->m_id);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, blas.m_buf_indices->m_id);

	glBindBufferBase(GL_UNIFORM_BUFFER, 3, constant_camera.m_id);
	glBindBufferBase(GL_UNIFORM_BUFFER, 4, prim.m_constant_model.m_id);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, prim.index_buf->m_id);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, prim.normal_buf->m_id);

	glBindImageTexture(0, tex_test.m_tex->tex_id, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_R8);
	glBindImageTexture(1, tex_depth.m_tex->tex_id, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_R8);
	glBindImageTexture(2, tex_normal.m_tex->tex_id, 0, GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA8);

	glm::ivec2 blocks = { (width + 31) / 32, (height + 1) / 2 };
	glDispatchCompute(blocks.x, blocks.y, 1);

	glUseProgram(0);

	std::vector<uint8_t> test(width * height);
	std::vector<uint8_t> depth(width * height);
	std::vector<glm::u8vec3> normal(width * height);

	{
		glBindTexture(GL_TEXTURE_2D, tex_test.m_tex->tex_id);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, test.data());
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	{
		glBindTexture(GL_TEXTURE_2D, tex_depth.m_tex->tex_id);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RED, GL_UNSIGNED_BYTE, depth.data());
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	{
		glBindTexture(GL_TEXTURE_2D, tex_normal.m_tex->tex_id);
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, normal.data());
		glBindTexture(GL_TEXTURE_2D, 0);
	}

	{
		FILE* fp = fopen("dmp_test.raw", "wb");
		fwrite(test.data(), 1, width * height, fp);
		fclose(fp);
	}

	{
		FILE* fp = fopen("dmp_depth.raw", "wb");
		fwrite(depth.data(), 1, width * height, fp);
		fclose(fp);
	}

	{
		FILE* fp = fopen("dmp_normal.raw", "wb");
		fwrite(normal.data(), 3, width * height, fp);
		fclose(fp);
	}
}


int main()
{
	glfwInit();
	GLFWwindow* window = glfwCreateWindow(1280, 720, "Test", nullptr, nullptr);
	glfwMakeContextCurrent(window);
	glewInit();

	test2();

	return 0;
}
