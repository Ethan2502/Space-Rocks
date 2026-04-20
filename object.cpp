#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#define G 50.0
#define DT 0.01

struct Vector {
    double x, y, z;
    Vector operator+(const Vector& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    Vector operator-(const Vector& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    Vector operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    Vector operator/(double scalar) const {
        return {x / scalar, y / scalar, z / scalar};
    }
    Vector operator+=(const Vector& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }
    double magnitude() const {
        return sqrt(x * x + y * y + z * z);
    }
};

struct Vector3 {
    Vector pos;
    Vector vel;
    Vector acc;


    Vector3 operator+(const Vector3& other) const {
        return {pos.x + other.pos.x, pos.y + other.pos.y, pos.z + other.pos.z,
                vel.x + other.vel.x, vel.y + other.vel.y, vel.z + other.vel.z,
                acc.x + other.acc.x, acc.y + other.acc.y, acc.z + other.acc.z};
    }

    Vector3 operator*(double scalar) const {
        return {pos.x * scalar, pos.y * scalar, pos.z * scalar,
                vel.x * scalar, vel.y * scalar, vel.z * scalar,
                acc.x * scalar, acc.y * scalar, acc.z * scalar};
    }

    Vector3 operator/(double scalar) const {
        return {pos.x / scalar, pos.y / scalar, pos.z / scalar,
                vel.x / scalar, vel.y / scalar, vel.z / scalar,
                acc.x / scalar, acc.y / scalar, acc.z / scalar};
    }

    Vector3 operator-(const Vector3& other) const {
        return {pos.x - other.pos.x, pos.y - other.pos.y, pos.z - other.pos.z,
                vel.x - other.vel.x, vel.y - other.vel.y, vel.z - other.vel.z,
                acc.x - other.acc.x, acc.y - other.acc.y, acc.z - other.acc.z};
    }
    Vector3 operator+=(const Vector3& other) {
        pos.x += other.pos.x; pos.y += other.pos.y; pos.z += other.pos.z;
        vel.x += other.vel.x; vel.y += other.vel.y; vel.z += other.vel.z;
        acc.x += other.acc.x; acc.y += other.acc.y; acc.z += other.acc.z;
        return *this;
    }
   
};

struct Body {
    double mass;
    double radius;
    Vector pos;
    Vector vel;
    Vector acc;
};

struct Node {
    double totalMass;
    Vector3 centerOfMass;
    Vector3 center;
    double width;

    Body* body = nullptr;
    Node* children[8] = {nullptr};
    
    Node(Vector3 c, double w) : center(c), width(w) {}

    bool isExternal() {
        for (int i = 0; i < 8; i++) if (children[i]) return false;
        return true;
    }

    ~Node() {
        for (int i = 0; i < 8; ++i) {
            if (children[i] != nullptr) {
                delete children[i]; // Recursive call to the child's destructor
            }
        }
    }
    
};

Vector vector_sub(Vector a, Vector b) {
    return (Vector){a.x - b.x, a.y - b.y, a.z - b.z};
}

double vector_mag(Vector v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector vector_scale(Vector v, double s) {
    return (Vector){v.x * s, v.y * s, v.z * s};
}

double calculateDistance(Vector a, Vector b) {
    // Vectorial separation: r = b - a
    Vector r_vec = vector_sub(b, a);
    // Magnitude: sqrt(dx^2 + dy^2 + dz^2)
    return vector_mag(r_vec);
}

void compute_accelerations(Body *bodies, int n) {
    for (int i = 0; i < n; i++) {
        bodies[i].acc = (Vector){0, 0, 0};
        for (int j = 0; j < n; j++) {
            if (i == j) continue;

            Vector r_vec = vector_sub(bodies[j].pos, bodies[i].pos);
            double r_mag = vector_mag(r_vec);

            double acc_mag = (G * bodies[j].mass) / (r_mag * r_mag * r_mag);

            bodies[i].acc.x += r_vec.x * acc_mag;
            bodies[i].acc.y += r_vec.y * acc_mag;
            bodies[i].acc.z += r_vec.z * acc_mag;

        }
    }
}

void applyGravity(Body* b, double otherMass, Vector otherPos) {
    Vector r_vec = {otherPos.x - b->pos.x, otherPos.y - b->pos.y, otherPos.z - b->pos.z};
    double r_mag = vector_mag(r_vec);

    // Prevent "garbage" results from division by zero or extremely close encounters
    if (r_mag < 1e-5) return;

    // acc = (G * M_other) / r^2
    // We multiply by (r_vec / r_mag) to get the direction, resulting in r_mag^3 denominator
    double acc_mag = (G * otherMass) / (r_mag * r_mag * r_mag);

    b->acc.x += r_vec.x * acc_mag;
    b->acc.y += r_vec.y * acc_mag;
    b->acc.z += r_vec.z * acc_mag;
}

void applyForceFromAggregate(Body* target, double nodeTotalMass, Vector3 nodeCenterOfMass) {
    // Approximation: Treat the entire cell as one large particle
    applyGravity(target, nodeTotalMass, nodeCenterOfMass.pos);
}

void applyDirectForce(Body* target, Body* source) {
    // Standard point-to-point gravity between two individual bodies
    applyGravity(target, source->mass, source->pos);
}

void update_positions(Body *bodies, int n) {
    for (int i = 0; i < n; i++) {
        //new vel
        bodies[i].vel.x += bodies[i].acc.x * DT;
        bodies[i].vel.y += bodies[i].acc.y * DT;
        bodies[i].vel.z += bodies[i].acc.z * DT;

        bodies[i].pos.x += bodies[i].vel.x * DT;
        bodies[i].pos.y += bodies[i].vel.y * DT;
        bodies[i].pos.z += bodies[i].vel.z * DT;
    }
}

void handleCollisions(Body& b1, Body& b2) {
    double dist = (b1.pos - b2.pos).magnitude();
    // If objects touch or pass through in one time step [4]
    if (dist < (b1.radius + b2.radius)) {
        // Conserve Momentum: v_new = (m1*v1 + m2*v2) / (m1+m2) [5]
        double totalMass = b1.mass + b2.mass;
        b1.vel = (b1.vel * b1.mass + b2.vel * b2.mass) / totalMass;
        b1.mass = totalMass;
        // Logic should follow to remove b2 from the simulation list
    }
}




double calculateTemperature(double distanceInAU, double albedo) {
    // tp approx 279 * (1-A)^0.25 * rp^-0.5 [11]
    // Albedo (A) is the fraction of light reflected (e.g., 0.4 for Earth) [11, 12]
    double temperature = 279.0 * pow(1.0 - albedo, 0.25) * pow(distanceInAU, -0.5);
    return temperature;
}


Vector3 calculateNewCenter(Node* node, int octant) {
    double offset = node->width / 4.0;
    Vector3 newCenter = node->center;

    if (octant & 1) newCenter.pos.x += offset; else newCenter.pos.x -= offset;
    if (octant & 2) newCenter.pos.y += offset; else newCenter.pos.y -= offset;
    if (octant & 4) newCenter.pos.z += offset; else newCenter.pos.z -= offset;

    return newCenter;
}


int getOctant(Node* node, Vector pos) {
    int octant = 0;
    if (pos.x >= node->center.pos.x) octant |= 1;
    if (pos.y >= node->center.pos.y) octant |= 2;
    if (pos.z >= node->center.pos.z) octant |= 4;
    return octant;
}


void subdivide(Node* node) {
    for (int i = 0; i < 8; i++) {
        Vector3 newCenter = calculateNewCenter(node, i);
        node->children[i] = new Node(newCenter, node->width / 2.0);
    }
}

void insert(Node* node, Body* body) {
    if (node->isExternal()) {
        if (node->body == nullptr) {
            node->body = body;
            node->totalMass = body->mass;
            node->centerOfMass.pos = body->pos;

            return;
        } else {
            Body* existingBody = node->body;
            node->body = nullptr;

            subdivide(node);
            insert(node, existingBody);
        }
    }

    double m1 = node->totalMass;
    double m2 = body->mass;
    node->centerOfMass = (node->centerOfMass * node->totalMass + Vector3{body->pos, {0, 0, 0}, {0, 0, 0}} * body->mass) / (m1 + m2);
    node->totalMass = m1 + m2;

    int octant = getOctant(node, body->pos);
    
    if (!node->children[octant]) {
        node->children[octant] = new Node(calculateNewCenter(node, octant), node->width / 2.0);
    }
    insert(node->children[octant], body);
}

void calculateForce(Node* node, Body* b, double theta) {
    // Skip if the node is empty
    if (node->totalMass == 0) return;

    if (node->isExternal()) {
        // CASE A: External node (leaf)
        // If it contains a body and it's not the body we are calculating for
        if (node->body != nullptr && node->body != b) {
            applyDirectForce(b, node->body); // Standard point-to-point gravity
        }
    } else {
        // CASE B: Internal node
        double s = node->width;
        double d = calculateDistance(b->pos, node->centerOfMass.pos);

        if (s / d < theta) {
            // The node is far away; treat all contained bodies as one large particle
            applyForceFromAggregate(b, node->totalMass, node->centerOfMass);
        } else {
            // The node is too close; recurse into all eight children
            for (int i = 0; i < 8; i++) {
                if (node->children[i]) {
                    calculateForce(node->children[i], b, theta);
                }
            }
        }
    }
}


void buildOctree() {

    Node* root = new Node((Vector3){{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 100.0); // Example center and width
    Body* bodies; // Assume this is initialized with your bodies
    int n; // Number of bodies
    for (int i = 0; i < n; i++) {
        insert(root, &bodies[i]);
    }
    delete root; // Clean up the octree after use
}

void stepForward(Body* bodies, int n, double dt) {
    for (int i = 0; i < n; i++) {
        // Euler integration: simple but requires small dt for stability [4, 8]
        bodies[i].pos += bodies[i].vel * dt;
        bodies[i].vel += bodies[i].acc * dt;
    }
}
#include <vector>

void handleAllCollisions(std::vector<Body>& bodies) {
    for (size_t i = 0; i < bodies.size(); i++) {
        for (size_t j = i + 1; j < bodies.size(); j++) {
            double dist = calculateDistance(bodies[i].pos, bodies[j].pos);
            if (dist < (bodies[i].radius + bodies[j].radius)) {
                // Merge j into i
                double totalMass = bodies[i].mass + bodies[j].mass;
                bodies[i].vel = (bodies[i].vel * bodies[i].mass + bodies[j].vel * bodies[j].mass) / totalMass;
                bodies[i].pos = (bodies[i].pos * bodies[i].mass + bodies[j].pos * bodies[j].mass) / totalMass;
                bodies[i].mass = totalMass;
                bodies[i].radius = std::max(bodies[i].radius, bodies[j].radius) + 0.1;
                
                // Remove j
                bodies.erase(bodies.begin() + j);
                j--; // adjust index after erasing
            }
        }
    }
}

int main() {
    std::vector<Body> bodies;
    
    // Star
    bodies.push_back({1000.0, 10.0, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}});

    // Planet 1
    bodies.push_back({1.0, 1.0, {100.0, 0.0, 0.0}, {0.0, 3.16227766, 0.0}, {0.0, 0.0, 0.0}});

    // Planet 2
    bodies.push_back({2.0, 1.5, {-150.0, 0.0, 0.0}, {0.0, -2.581988, 0.0}, {0.0, 0.0, 0.0}});
    
    // Asteroid
    bodies.push_back({0.1, 0.5, {0.0, 50.0, 0.0}, {-4.47213, 0.0, 0.0}, {0.0, 0.0, 0.0}});

    int steps = 100;

    printf("Starting simulation...\n");
    for (int step = 0; step < steps; step++) {
        // Reset accelerations
        for (auto& b : bodies) {
            b.acc = {0, 0, 0};
        }

        // Use Octree (Barnes-Hut)
        Node* root = new Node((Vector3){{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, 1000.0);
        for (auto& b : bodies) {
            insert(root, &b);
        }

        // Calculate forces
        for (auto& b : bodies) {
            calculateForce(root, &b, 0.5); // Theta = 0.5
        }
        delete root;

        update_positions(bodies.data(), bodies.size());
        handleAllCollisions(bodies);
        
        if (step % 10 == 0) {
            printf("Step %d: Total Bodies: %zu\n", step, bodies.size());
            if (bodies.size() > 1) {
                printf("  Planet 1 pos: (%.2f, %.2f, %.2f)\n", bodies[1].pos.x, bodies[1].pos.y, bodies[1].pos.z);
            }
        }
    }
    printf("Simulation complete.\n");

    return 0;
}
