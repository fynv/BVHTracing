#include <bvh/bvh.hpp>
#include <bvh/vector.hpp>
#include <bvh/triangle.hpp>
#include <bvh/sweep_sah_builder.hpp>

#include <bvh/ray.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>

#include <glm.hpp>
#include <gtc/matrix_transform.hpp>
#include <unordered_map>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "crc64.h"

const double PI = 3.14159265359;
const double DEG2RAD = PI / 180.0;

struct Primitive
{
	glm::mat4 model_matrix;

	int num_pos = 0;
	int num_face = 0;

	std::vector<glm::vec4> pos;
	std::vector<glm::vec4> norm;
	std::vector<glm::ivec3> faces;

	Primitive()
	{
		model_matrix = glm::identity<glm::mat4>();
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
	}
};

class BLAS
{
public:
	struct Intersection {
		int triangle_index = -1;
		float t = -1.0f;
		float u = 0.0f;
		float v = 0.0f;
		float distance() const { return t; }
	};

	using ScalarType = float;
	using IntersectionType = Intersection;

	BLAS(BLAS&&) = default;
	BLAS(const Primitive* primitive);
	~BLAS();

	bvh::Vector3<float> center() const;
	bvh::BoundingBox<float> bounding_box() const;
	std::optional<Intersection> intersect(const bvh::Ray<float>& ray, int culling) const;

	BLAS& operator =(BLAS&&) = default;

private:
	typedef bvh::Triangle<float, true, true> PrimitiveType;
	typedef bvh::ClosestPrimitiveIntersector<bvh::Bvh<float>, PrimitiveType> IntersectorType;
	typedef bvh::SingleRayTraverser<bvh::Bvh<float>> TraversorType;

	std::vector<PrimitiveType> m_triangles;
	bvh::BoundingBox<float> m_bounding_box;
	std::unique_ptr<bvh::Bvh<float>> m_bvh;
	std::unique_ptr<IntersectorType> m_intersector[3];
	std::unique_ptr<TraversorType> m_traverser;
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

		m_triangles.emplace_back(PrimitiveType(
			bvh::Vector3<float>(v0.x, v0.y, v0.z),
			bvh::Vector3<float>(v1.x, v1.y, v1.z),
			bvh::Vector3<float>(v2.x, v2.y, v2.z)
		));
	}

	auto [bboxes, centers] = bvh::compute_bounding_boxes_and_centers(m_triangles.data(), m_triangles.size());
	m_bounding_box = bvh::compute_bounding_boxes_union(bboxes.get(), m_triangles.size());

	m_bvh = std::unique_ptr<bvh::Bvh<float>>(new bvh::Bvh<float>);
	bvh::SweepSahBuilder<bvh::Bvh<float>> builder(*m_bvh);
	builder.build(m_bounding_box, bboxes.get(), centers.get(), m_triangles.size());

	m_intersector[0] = std::unique_ptr<IntersectorType>(new IntersectorType(*m_bvh, m_triangles.data()));
	m_intersector[0]->culling = 0;
	m_intersector[1] = std::unique_ptr<IntersectorType>(new IntersectorType(*m_bvh, m_triangles.data()));
	m_intersector[1]->culling = 1;
	m_intersector[2] = std::unique_ptr<IntersectorType>(new IntersectorType(*m_bvh, m_triangles.data()));
	m_intersector[2]->culling = 2;
	m_traverser = std::unique_ptr<TraversorType>(new TraversorType(*m_bvh));

}

BLAS::~BLAS()
{

}

bvh::Vector3<float> BLAS::center() const
{
	return m_bounding_box.center();
}

bvh::BoundingBox<float> BLAS::bounding_box() const
{
	return m_bounding_box;
}


std::optional<BLAS::Intersection> BLAS::intersect(const bvh::Ray<float>& ray, int culling) const
{
	auto hit = m_traverser->traverse(ray, *m_intersector[culling]);
	if (hit.has_value())
	{
		auto intersection = hit->intersection;
		Intersection ret;
		ret.triangle_index = hit->primitive_index;
		ret.t = intersection.t;
		ret.u = intersection.u;
		ret.v = intersection.v;
		return std::make_optional<Intersection>(ret);
	}
	return std::nullopt;
}

int main()
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

	BLAS blas(&prim);

	glm::mat4 invProjection = glm::inverse(projection);
	glm::mat4 invView = glm::inverse(view);
	glm::vec3 eyePos = glm::vec3(invView[3]);
	glm::ivec2 viewport_size(width, height);

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

			bvh::Ray<float> bvh_ray = bvh::Ray<float>(
				bvh::Vector3<float>(eyePos.x, eyePos.y, eyePos.z),
				bvh::Vector3<float>(dir.x, dir.y, dir.z)
				);

			float d = 1.0f;
			glm::vec3 norm = glm::vec3(0.0f, 0.0f, 1.0f);

			auto intersection = blas.intersect(bvh_ray, 0);
			if (intersection.has_value())
			{
				d = (intersection->t - 2.0f) / 5.0f;
				if (d < 0.0f) d = 0.0f;
				if (d > 1.0f) d = 1.0f;

				int face_id = intersection->triangle_index;
				float u = intersection->u;
				float v = intersection->v;
				glm::ivec3 face = prim.faces[face_id];
				glm::vec4 norm0 = prim.norm[face.x];
				glm::vec4 norm1 = prim.norm[face.y];
				glm::vec4 norm2 = prim.norm[face.z];
				glm::vec4 normal = (1.0f - u - v) * norm0 + u * norm1 + v * norm2;

				norm = glm::vec3(normalMat * normal);
			}
			depth[x + y * width] = (uint8_t)(d * 255.0f + 0.5f);
			normal[x + y * width] = glm::u8vec3((norm * 0.5f + 0.5f) * 255.0f + 0.5f);

		}
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

	return 0;
}
