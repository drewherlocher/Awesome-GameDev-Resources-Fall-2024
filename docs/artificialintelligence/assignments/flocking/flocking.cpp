#include <iostream>
#include <iomanip>
#include <vector>
#include <utility>
#include <cmath>

using namespace std;

struct Vector2 {
  double x=0, y=0;
  Vector2() : x(0), y(0){};
  Vector2(double x, double y) : x(x), y(y){};
  Vector2(const Vector2& v) = default;

  // unary operations
  Vector2 operator-() const { return {-x, -y}; }
  Vector2 operator+() const { return {x, y}; }

  // binary operations
  Vector2 operator-(const Vector2& rhs) const { return {x - rhs.x, y - rhs.y}; }
  Vector2 operator+(const Vector2& rhs) const { return {x + rhs.x, y + rhs.y}; }
  Vector2 operator*(const double& rhs) const { return {x * rhs, y * rhs}; }
  friend Vector2 operator*(const double& lhs, const Vector2& rhs) { return {lhs * rhs.x, lhs * rhs.y}; }
  Vector2 operator/(const double& rhs) const { return {x / rhs, y / rhs}; }
  Vector2 operator/(const Vector2& rhs) const { return {x / rhs.x, y / rhs.y}; }
  bool operator!=(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() >= 1.0e-6; };
  bool operator==(const Vector2& rhs) const { return (*this - rhs).sqrMagnitude() < 1.0e-6; };

  // assignment operation
  Vector2& operator=(Vector2 const& rhs) = default;
  Vector2& operator=(Vector2&& rhs) = default;

  // compound assignment operations
  Vector2& operator+=(const Vector2& rhs) {
    x += rhs.x;
    y += rhs.y;
    return *this;
  }
  Vector2& operator-=(const Vector2& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    return *this;
  }
  Vector2& operator*=(const double& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
  }
  Vector2& operator/=(const double& rhs) {
    x /= rhs;
    y /= rhs;
    return *this;
  }
  Vector2& operator*=(const Vector2& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
  }
  Vector2& operator/=(const Vector2& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
  }

  double sqrMagnitude() const { return x * x + y * y; }
  double getMagnitude() const { return sqrt(sqrMagnitude()); }
  static double getMagnitude(const Vector2& vector) { return vector.getMagnitude(); }

  static double Distance(const Vector2& a, const Vector2& b) { return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); };
  double Distance(const Vector2& b) const { return sqrt((x - b.x) * (x - b.x) + (y - b.y) * (y - b.y)); };
  static double DistanceSquared(const Vector2& a, const Vector2& b) { return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y); };
  double DistanceSquared(const Vector2& b) const { return (x - b.x) * (x - b.x) + (y - b.y) * (y - b.y); };

  static Vector2 normalized(const Vector2& v) { return v.normalized(); };
  Vector2 normalized() const {
    auto magnitude = getMagnitude();

    // If the magnitude is not null
    if (magnitude > 0.)
      return Vector2(x, y) / magnitude;
    else
      return {x, y};
  };

  static const Vector2 zero;
};

const Vector2 Vector2::zero = {0, 0};

struct Boid {
  Boid(const Vector2& pos, const Vector2& vel): position(pos), velocity(vel){};
  Boid():position({0,0}), velocity({0,0}){};
  Vector2 position;
  Vector2 velocity;
};

// Checks the boid index
bool isValidBoidIndex(const vector<Boid>& boids, int boidAgentIndex) {
  if (boidAgentIndex < 0 || boidAgentIndex>= boids.size()) {
    cerr << "Error: boidAgentIndex out of range." << endl;
    return true;
  }
  return false;
}

struct Cohesion {
  double radius;
  double k;

  Cohesion() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    double totalWeight = 0.0;
    Vector2 cohesionForce = Vector2::zero;
    Vector2 centerOfMass = Vector2::zero;
    Vector2 averageCenterOfMass = Vector2::zero;

    if (isValidBoidIndex(boids, boidAgentIndex)) {
      return Vector2::zero;
    }

    // Go through the boids to calculate center of mass
    for (const auto& boid : boids) {
      double distance = Vector2::Distance(boids[boidAgentIndex].position, boid.position);
      if (distance > 0 && distance < radius) {
        double weight = 1 / distance;
        centerOfMass += boid.position * weight;
        totalWeight += weight;
      }
    }

    // Calculate the center of mass if there are neighbors
    if (totalWeight > 0) {
      averageCenterOfMass = centerOfMass / totalWeight;
      cohesionForce = averageCenterOfMass - boids[boidAgentIndex].position;
      cohesionForce = cohesionForce.normalized() * k;
      return cohesionForce;
    }

    return Vector2::zero;
  }
};

struct Alignment {
  double radius;
  double k;

  Alignment() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    Vector2 totalVelocity = Vector2::zero;
    Vector2 averageVelocity = Vector2::zero;
    Vector2 alignmentForce = Vector2::zero;
    int neighbors = 0;

    if (isValidBoidIndex(boids, boidAgentIndex)) {
      return Vector2::zero;
    }

    for (const auto& boid : boids) {
      double distance = Vector2::Distance(boids[boidAgentIndex].position, boid.position);
      if (distance > 0 && distance < radius) {
        totalVelocity += boid.velocity;
        neighbors++;
      }
    }

    if (neighbors > 0) {
      averageVelocity = totalVelocity / neighbors;
      alignmentForce = averageVelocity - boids[boidAgentIndex].velocity;
      alignmentForce = alignmentForce.normalized() * k;
      return alignmentForce;
    }
    return Vector2::zero;
  }
};

struct Separation {
  double radius;
  double k;
  double maxForce;

  Separation() = default;

  Vector2 ComputeForce(const vector<Boid>& boids, int boidAgentIndex) {
    Vector2 separationForce = Vector2::zero;
    int neighbors = 0;
    if (isValidBoidIndex(boids, boidAgentIndex)) {
      return Vector2::zero;
    }
    for (const auto& boid : boids) {
      if (&boid != &boids[boidAgentIndex]) {
        double distance = Vector2::Distance(boids[boidAgentIndex].position, boid.position);
        if (distance > 0 && distance < radius) {
          Vector2 direction = boids[boidAgentIndex].position - boid.position;
          separationForce += direction.normalized() / distance;
          neighbors++;
        }
      }
    }

    if (neighbors > 0) {
      separationForce = separationForce.normalized() * k;
      if (separationForce.getMagnitude() > maxForce) {
        separationForce = separationForce.normalized() * maxForce;
      }
      return separationForce;
    }
  return Vector2::zero;
  }
};

// feel free to edit this main function to meet your needs
int main() {
  // Variable declaration
  Separation separation{};
  Alignment alignment{};
  Cohesion cohesion{};
  int numberOfBoids;
  string line; // for reading until EOF
  vector<Boid> currentState, newState;
  // Input Reading
  cin >> cohesion.radius >> separation.radius >> separation.maxForce >> alignment.radius >> cohesion.k >> separation.k >> alignment.k >> numberOfBoids;
  for (int i = 0; i < numberOfBoids; i++)
  {
    Boid b;
    cin >> b.position.x >> b.position.y >> b.velocity.x >> b.velocity.y;
    //cout << "b.y: " << b.y << endl;
    currentState.push_back(b);
    newState.push_back(b);
  }
 // cohesion.PrintDebug();

  cin.ignore(numeric_limits<streamsize>::max(), '\n');
  // Final input reading and processing
  // todo: edit this. probably my code will be different than yours.
  while (getline(cin, line)) { // game loop
    double deltaT;
    try {
      if (!line.empty()) {
        deltaT = stod(line);  // Convert line to double
      } else {
        cerr << "Error: Empty line encountered while reading deltaT" << endl;
        return 1;
      }
    } catch (const std::invalid_argument& e) {
      cerr << "Error: " << e.what() << endl;
      return 1;
    }

    // Use double buffer! you should read from the current and store changes in the new state.
    currentState = newState;
    vector<Vector2> allForces = vector<Vector2>(numberOfBoids, {0, 0});
    // Compute Forces
    for (int i = 0; i < numberOfBoids; i++)  // for every boid
    {
      for (int j = 0; j < numberOfBoids; j++)  // for every boid combination. Pre-processing loop.
      {
        // Process Cohesion Forces
        auto dist = (currentState[i].position-currentState[j].position).getMagnitude();
        if (i != j && dist <= cohesion.radius) {
          allForces[i] += cohesion.ComputeForce(currentState, i);
        }
        // Process Separation Forces
        if (i != j && dist <= separation.radius) {
          allForces[i] += separation.ComputeForce(currentState, i);
        }
        // Process Alignment Forces
        if (i != j && dist <= alignment.radius) {
          allForces[i] += alignment.ComputeForce(currentState, i);
        }
      }
    }
    // Tick Time and Output
    // todo: edit this. probably my code will be different than yours.
    cout << fixed << setprecision(3);  // set 3 decimal places precision for output
    for (int i = 0; i < numberOfBoids; i++) // for every boid
    {
      newState[i].velocity += allForces[i] * deltaT;
      newState[i].position += currentState[i].velocity * deltaT;
      cout << newState[i].position.x << " " << newState[i].position.y << " "
           << newState[i].velocity.x << " " << newState[i].velocity.y << endl;
    }
  }

  return 0;
}
