//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TODO Implement Path Tracing Algorithm here
    Vector3f hitcolor(0.0);
    Intersection intersection = intersect(ray);

    if (intersection.emit.norm() > 0)
        hitcolor = Vector3f(1.0);
    else if (intersection.happened) {
        Vector3f L_dir(0.0), L_indir(0.0);

        Material *m = intersection.m;
        Vector3f p = intersection.coords;
        Vector3f N = intersection.normal;
        Vector3f wo = normalize(-ray.direction);

        // Direct light
        Intersection inter;
        float pdf_light;
        sampleLight(inter, pdf_light);
        Vector3f x = inter.coords;
        Vector3f NN = inter.normal;
        Vector3f emit = inter.emit;
        Vector3f ws = normalize(x - p);
        // Not blocked
        if ((intersect(Ray(p + EPSILON * ws, ws)).coords - x).norm() < 0.01) {
            L_dir = emit * m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / pow((x - p).norm(), 2) / pdf_light;
        }

        // Indirect light
        float P_RR = get_random_float();
        if (P_RR < RussianRoulette) {
            Vector3f wi = m->sample(wo, N);
            // L_indir = castRay(Ray(p + EPSILON * wi, wi), depth + 1) * m->eval(wi, wo, N) * dotProduct(wi, N) / m->pdf(wi, wo, N) / RussianRoulette;
            L_indir = castRay(Ray(p + EPSILON * wi, wi), depth + 1) * m->eval(wo, wi, N) * dotProduct(wi, N) / m->pdf(wo, wi, N) / RussianRoulette;
        }

        hitcolor = L_dir + L_indir;
    }

    return hitcolor;
}
