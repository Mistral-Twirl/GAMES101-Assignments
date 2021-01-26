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
    // TO DO Implement Path Tracing Algorithm here

/*
shade (p, wo)
    sampleLight ( inter , pdf_light )
    Get x, ws , NN , emit from inter
    Shoot a ray from p to x
    If the ray is not blocked in the middle
        L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws , NN) / |x-p |^2 / pdf_light
    
    L_indir = 0.0
    Test Russian Roulette with probability RussianRoulette
    wi = sample (wo , N)
    Trace a ray r(p, wi)
    If ray r hit a non - emitting object at q
        L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N) / pdf (wo , wi , N) / RussianRoulette
    
    Return L_dir + L_indir
*/

    Vector3f color(0.0f, 0.0f, 0.0f);
    
    Intersection interRay = Scene::intersect(ray);

    if(interRay.emit.norm() > 0){ // the ray hits the light source
        color = Vector3f(1.0f, 1.0f, 1.0f);
    }
    else if(interRay.happened){ // the ray hits objects
        Material* m = interRay.m;
        Vector3f p = interRay.coords; 
        Vector3f wo = - ray.direction;
        Vector3f N = normalize(interRay.normal);

        Intersection inter;
        float pdf_light;
        sampleLight(inter, pdf_light);
        Vector3f x = inter.coords;
        Vector3f ws = normalize(x - p);
        Vector3f NN = normalize(inter.normal);

/*  
    Shoot a ray from p to x
    If the ray is not blocked in the middle
    L_dir = emit * eval (wo , ws , N) * dot (ws , N) * dot (ws , NN) / |x-p |^2 / pdf_light
*/
       
        Vector3f L_dir(0.0f, 0.0f, 0.0f);
        Intersection interDirLight = intersect(Ray(p, ws));
        bool unblocked = ((interDirLight.coords - inter.coords).norm() < 0.01);
        if (unblocked){
            L_dir = inter.emit * m->eval(ws, wo, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / dotProduct((x - p), (x - p)) / pdf_light;
        }

/*
    Test Russian Roulette with probability RussianRoulette
    wi = sample (wo , N)
    Trace a ray r(p, wi)
    If ray r hit a non - emitting object at q
        L_indir = shade (q, wi) * eval (wo , wi , N) * dot (wi , N) / pdf (wo , wi , N) / RussianRoulette
*/
        Vector3f L_indir(0.0f, 0.0f, 0.0f);
        float p_RR = get_random_float();

        if (p_RR < Scene::RussianRoulette){
            Vector3f wi = m->sample(wo, N);
            
            L_indir = castRay(Ray(p, wi), depth) * m->eval(wi, wo, N) * dotProduct(wi, N) / m->pdf(wi, wo, N) / Scene::RussianRoulette;
        }
        
        L_indir = L_indir * 1.0f / Scene::RussianRoulette;

        color = L_dir + L_indir;
    }
return color;

}