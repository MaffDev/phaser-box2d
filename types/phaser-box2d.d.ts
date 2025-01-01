declare namespace Aabb { }

/**
 * Checks if an AABB is valid by verifying:
 * 1. The AABB object exists
 * 2. The width (upperBoundX - lowerBoundX) is non-negative
 * 3. The height (upperBoundY - lowerBoundY) is non-negative
 * 4. All coordinate values are valid numbers
 * @param aabb - The AABB to validate
 * @returns True if the AABB exists and has valid dimensions and coordinates
 */
declare function b2AABB_IsValid(aabb: b2AABB): boolean;

declare namespace Allocate { }

declare namespace BitSet { }

declare namespace BlockArray { }

declare namespace Body { }

/**
 * Creates a new physics body in the specified world based on the provided body definition.
 * The body is added to the appropriate solver set based on its type and state.
 * The function initializes the body's physical properties including position, rotation,
 * velocities, damping values and other simulation parameters.
 * @param worldId - The ID of the world to create the body in
 * @param def - The body definition containing initialization parameters
 * @returns The ID of the newly created body
 */
declare function b2CreateBody(worldId: b2WorldId, def: b2BodyDef): b2BodyId;

/**
 * Destroys a body and all associated joints, contacts, shapes, and chains in the physics world.
 * The function cleans up all resources and removes the body from the simulation system.
 * @param bodyId - The identifier of the body to destroy
 */
declare function b2DestroyBody(bodyId: b2BodyId): void;

/**
 * Retrieves the current number of contacts associated with the specified body.
 * The function first validates the world reference before accessing the body's contact count.
 * @param bodyId - The identifier for the body to query
 * @returns The number of contacts associated with the body. Returns 0 if the world is invalid.
 */
declare function b2Body_GetContactCapacity(bodyId: b2BodyId): number;

/**
 * Retrieves contact data for a specified body. For each active contact, stores the shape IDs
 * of both bodies involved and the contact manifold. Only stores contacts that have the
 * touching flag set.
 * @param bodyId - The ID of the body to get contact data from
 * @param contactData - Array to store the contact data
 * @param capacity - Maximum number of contacts to retrieve
 * @returns The number of contacts stored in contactData
 */
declare function b2Body_GetContactData(bodyId: b2BodyId, contactData: b2ContactData[], capacity: number): number;

/**
 * For bodies with no shapes, returns an AABB containing only the body's position.
 * For bodies with shapes, computes the union of AABBs of all shapes attached to the body.
 * @param bodyId - The identifier for the body whose AABB is to be computed.
 * @returns An AABB that encompasses the body and all its shapes. Returns an empty AABB if the world is not found.
 */
declare function b2Body_ComputeAABB(bodyId: b2BodyId): b2AABB;

/**
 * Retrieves the current position component of a body's transform from the physics world.
 * The position is returned as a b2Vec2 representing the body's location in world space.
 * @param bodyId - The identifier for the body whose position is being queried.
 * @returns The position vector of the body in world coordinates.
 */
declare function b2Body_GetPosition(bodyId: b2BodyId): b2Vec2;

/**
 * Retrieves the rotation component (q) from the body's transform. The returned b2Rot
 * object represents the body's angular orientation in 2D space.
 * @param bodyId - The identifier for the body whose rotation is being queried.
 * @returns A rotation object containing the cosine and sine of the body's angle.
 */
declare function b2Body_GetRotation(bodyId: b2BodyId): b2Rot;

/**
 * Retrieves the current transform (position and rotation) of a physics body
 * from the specified Box2D world using the body's identifier.
 * @param bodyId - The identifier object for the body, containing world reference.
 * @param bodyId.world0 - The world identifier.
 * @returns The body's transform containing:
 * - position {b2Vec2} The position vector (x, y)
 * - rotation {b2Rot} The rotation values (c, s)
 */
declare function b2Body_GetTransform(bodyId: {
    world0: number;
}): any;

/**
 * Takes a point given in world coordinates and converts it to the body's local
 * coordinate system by applying the inverse of the body's transform.
 * @param bodyId - The identifier for the body
 * @param worldPoint - A point in world coordinates
 * @returns The point expressed in the body's local coordinates
 */
declare function b2Body_GetLocalPoint(bodyId: b2BodyId, worldPoint: b2Vec2): b2Vec2;

/**
 * Takes a point given in body-local coordinates and converts it to world coordinates
 * using the body's current transform (position and rotation).
 * @param bodyId - The identifier for the body.
 * @param localPoint - A point in the body's local coordinates.
 * @returns The point transformed to world coordinates.
 */
declare function b2Body_GetWorldPoint(bodyId: b2BodyId, localPoint: b2Vec2): b2Vec2;

/**
 * Takes a vector defined in world coordinates and transforms it to be relative
 * to the body's local coordinate system by applying the inverse of the body's rotation.
 * @param bodyId - The identifier for the body
 * @param worldVector - A vector in world coordinates
 * @returns The vector transformed into the body's local coordinates
 */
declare function b2Body_GetLocalVector(bodyId: b2BodyId, worldVector: b2Vec2): b2Vec2;

/**
 * Transforms a vector from the body's local coordinate system to the world
 * coordinate system. This operation only performs rotation (not translation)
 * using the body's current rotation matrix.
 * @param bodyId - The identifier for the body
 * @param localVector - A vector in local body coordinates
 * @returns The vector transformed into world coordinates
 */
declare function b2Body_GetWorldVector(bodyId: b2BodyId, localVector: b2Vec2): b2Vec2;

/**
 * Sets the position and rotation of a body, updating its transform and associated shape AABBs.
 * The function updates the body's center, handles broad-phase movement, and adjusts collision bounds.
 * @param bodyId - The identifier of the body to transform
 * @param position - The new position vector for the body
 * @param [rotation] - The new rotation for the body. If undefined, keeps current rotation
 */
declare function b2Body_SetTransform(bodyId: b2BodyId, position: b2Vec2, rotation?: b2Rot): void;

/**
 * Retrieves the current linear velocity of a body from its state in the physics world.
 * If the body state cannot be found, returns a zero vector.
 * Linear velocity is often used for calculations (damping, direction, etc) and must be cloned to avoid unwanted effects in the Physics engine.
 * @param bodyId - The identifier for the body.
 * @returns The linear velocity vector of the body. Returns a zero vector (0,0) if the body state is null.
 */
declare function b2Body_GetLinearVelocity(bodyId: b2BodyId): b2Vec2;

/**
 * Retrieves the current angular velocity of a body from its state in the physics world.
 * Angular velocity represents how fast the body is rotating.
 * @param bodyId - The identifier for the body.
 * @returns The angular velocity in radians per second. Returns 0 if the body state cannot be retrieved.
 */
declare function b2Body_GetAngularVelocity(bodyId: b2BodyId): number;

/**
 * Sets the linear velocity of a body. If the body is static, the function returns without
 * making changes. If the new velocity has a non-zero magnitude, the body will be awakened.
 * The function will return early if the body state cannot be retrieved.
 * @param bodyId - The identifier of the body to modify.
 * @param linearVelocity - The new linear velocity vector to set.
 */
declare function b2Body_SetLinearVelocity(bodyId: b2BodyId, linearVelocity: b2Vec2): void;

/**
 * Sets the angular velocity of a body. If the body is static or has fixed rotation,
 * the function returns without making changes. The body is woken up if a non-zero
 * angular velocity is set.
 * @param bodyId - The identifier of the body to modify
 * @param angularVelocity - The new angular velocity in radians per second
 */
declare function b2Body_SetAngularVelocity(bodyId: b2BodyId, angularVelocity: number): void;

/**
 * Applies a force at a world point to a body. If the force is not applied at the
 * center of mass, it will generate a torque and affect angular motion.
 * @param bodyId - The identifier of the body to apply the force to
 * @param force - The world force vector
 * @param point - The world position of the point of application
 * @param wake - Whether to wake the body if it is sleeping
 */
declare function b2Body_ApplyForce(bodyId: b2BodyId, force: b2Vec2, point: b2Vec2, wake: boolean): void;

/**
 * Applies a force to the center of mass of a body. If the body is sleeping, it can optionally be woken up.
 * The force is only applied if the body is in the awake set.
 * @param bodyId - The identifier of the body to apply the force to
 * @param force - The world force vector to apply to the body's center
 * @param wake - Whether to wake the body if it is sleeping
 */
declare function b2Body_ApplyForceToCenter(bodyId: b2BodyId, force: b2Vec2, wake: boolean): void;

/**
 * Adds the specified torque to the body's total torque. If the wake parameter
 * is true and the body is sleeping, it will be awakened. The torque is only
 * applied if the body is in the awake set.
 * @param bodyId - The identifier of the body to apply torque to
 * @param torque - The amount of torque to apply
 * @param wake - Whether to wake the body if it's sleeping
 */
declare function b2Body_ApplyTorque(bodyId: b2BodyId, torque: number, wake: boolean): void;

/**
 * Applies a linear impulse to a body at a specified world point, affecting both its linear
 * and angular velocities.
 * @param bodyId - The identifier of the body to apply the impulse to
 * @param impulse - The world impulse vector to apply
 * @param point - The world position where the impulse is applied
 * @param wake - Whether to wake the body if it's sleeping
 */
declare function b2Body_ApplyLinearImpulse(bodyId: b2BodyId, impulse: b2Vec2, point: b2Vec2, wake: boolean): void;

/**
 * Applies a linear impulse to the center of mass of a body. If the body is sleeping,
 * it can optionally be woken up.
 * @param bodyId - The identifier of the body to apply the impulse to
 * @param impulse - The linear impulse vector to be applied
 * @param wake - Whether to wake the body if it's sleeping
 */
declare function b2Body_ApplyLinearImpulseToCenter(bodyId: b2BodyId, impulse: b2Vec2, wake: boolean): void;

/**
 * Applies an angular impulse to a body, affecting its angular velocity. The impulse is
 * scaled by the body's inverse inertia.
 * @param bodyId - The identifier of the body to apply the impulse to
 * @param impulse - The angular impulse to apply
 * @param wake - Whether to wake the body if it's sleeping
 */
declare function b2Body_ApplyAngularImpulse(bodyId: b2BodyId, impulse: number, wake: boolean): void;

/**
 * Retrieves the body type (static, kinematic, or dynamic) for a given body ID
 * by looking up the body in the physics world using the provided identifier.
 * @param bodyId - The identifier for the body to query.
 * @returns The type of the specified body.
 */
declare function b2Body_GetType(bodyId: b2BodyId): b2BodyType;

/**
 * Updates a body's type and handles all necessary simulation changes including:
 * - Destroying existing contacts
 * - Waking the body and connected bodies
 * - Unlinking and relinking joints
 * - Transferring the body between solver sets
 * - Updating broad-phase proxies
 * - Recalculating mass data
 * If the body is disabled or the type is unchanged, minimal processing occurs.
 * Special handling exists for transitions to/from static body type.
 * @param bodyId - The ID of the body to modify
 * @param type - The new body type to set (static, kinematic, or dynamic)
 */
declare function b2Body_SetType(bodyId: b2BodyId, type: b2BodyType): void;

/**
 * Associates arbitrary user data with a physics body. The user data can be
 * retrieved later and can be of any type. The body is located using its
 * world identifier and the user data is stored directly on the body object.
 * @param bodyId - The identifier of the body to modify.
 * @param userData - The user data to associate with the body.
 */
declare function b2Body_SetUserData(bodyId: b2BodyId, userData: any): void;

/**
 * Retrieves the custom user data that was previously attached to the specified body.
 * The function first gets the world from the body ID, then retrieves the full body
 * object, and finally returns its user data.
 * @param bodyId - The identifier for the body.
 * @returns The user data associated with the body.
 */
declare function b2Body_GetUserData(bodyId: b2BodyId): any;

/**
 * Retrieves the mass value from a body's simulation data by accessing the world
 * and body objects using the provided body identifier.
 * @param bodyId - The identifier for the body whose mass is to be retrieved.
 * @returns The mass of the body in kilograms.
 */
declare function b2Body_GetMass(bodyId: b2BodyId): number;

/**
 * Retrieves the rotational inertia value from a body's simulation data using the body's ID.
 * The inertia tensor represents the body's resistance to rotational acceleration.
 * @param bodyId - The ID of the body to get the inertia tensor from.
 * @returns The inertia tensor value of the body.
 */
declare function b2Body_GetInertiaTensor(bodyId: b2BodyId): number;

/**
 * Returns a copy of the body's local center of mass position vector.
 * The local center is expressed in the body's local coordinate system.
 * @param bodyId - The identifier for the body.
 * @returns The local center of mass position vector.
 */
declare function b2Body_GetLocalCenterOfMass(bodyId: b2BodyId): b2Vec2;

/**
 * Returns a copy of the body's center of mass position in world coordinates.
 * The returned vector is a clone of the internal state, preventing external
 * modification of the body's actual center position.
 * @param bodyId - The identifier for the body whose center of mass position is being queried.
 * @returns A vector representing the world position of the body's center of mass.
 */
declare function b2Body_GetWorldCenterOfMass(bodyId: b2BodyId): b2Vec2;

/**
 * Sets the mass properties of a body including mass, rotational inertia, and center of mass.
 * The function updates both the primary mass properties and derived values like inverse mass.
 * @param bodyId - The identifier for the body to modify
 * @param massData - An object containing:
 * - mass: The mass of the body (must be >= 0)
 * - rotationalInertia: The rotational inertia (must be >= 0)
 * - center: A b2Vec2 representing the local center of mass
 */
declare function b2Body_SetMassData(bodyId: b2BodyId, massData: b2MassData): void;

/**
 * Retrieves the mass properties of a body, including its total mass,
 * center of mass position, and rotational inertia.
 * @param bodyId - The identifier for the body whose mass data is being retrieved
 * @returns An object containing mass properties:
 * - mass: The total mass of the body
 * - center: A b2Vec2 representing the local center of mass
 * - rotationalInertia: The rotational inertia about the local center of mass
 */
declare function b2Body_GetMassData(bodyId: b2BodyId): b2MassData;

/**
 * This function retrieves the body from the world using its ID and updates its mass data
 * properties (mass, center of mass, and rotational inertia) based on the shapes attached to it.
 * If the world cannot be accessed, the function returns without performing any operations.
 * @param bodyId - The identifier for the body whose mass properties need to be updated.
 */
declare function b2Body_ApplyMassFromShapes(bodyId: b2BodyId): void;

/**
 * Sets the linear damping coefficient for a body, which reduces its linear velocity over time.
 * Linear damping is used to simulate air resistance or fluid friction.
 * @param bodyId - The identifier for the body to modify.
 * @param linearDamping - The new linear damping value. Must be non-negative.
 */
declare function b2Body_SetLinearDamping(bodyId: b2BodyId, linearDamping: number): void;

/**
 * Returns the linear damping coefficient that reduces the body's linear velocity.
 * Linear damping is used to reduce the linear velocity of the body in the absence
 * of forces. The damping parameter can be used to simulate fluid/air resistance.
 * @param bodyId - The identifier for the body.
 * @returns The linear damping coefficient of the body.
 */
declare function b2Body_GetLinearDamping(bodyId: b2BodyId): number;

/**
 * Sets the angular damping coefficient for a body, which reduces its angular velocity over time.
 * Angular damping is a value between 0 and infinity that reduces the body's angular velocity.
 * @param bodyId - The identifier for the target body.
 * @param angularDamping - The new angular damping value. Must be non-negative.
 */
declare function b2Body_SetAngularDamping(bodyId: b2BodyId, angularDamping: number): void;

/**
 * Returns the angular damping coefficient that reduces the body's angular velocity
 * over time. Angular damping is specified in the range [0,1].
 * @param bodyId - The identifier for the body.
 * @returns The angular damping coefficient of the body.
 */
declare function b2Body_GetAngularDamping(bodyId: b2BodyId): number;

/**
 * Sets a scale factor that modifies the effect of gravity on a specific body.
 * A value of 1.0 indicates normal gravity, 0.0 indicates no gravity, and negative
 * values reverse the effect of gravity on the body.
 * @param bodyId - The identifier for the body to modify.
 * @param gravityScale - The scaling factor to apply to gravity for this body.
 */
declare function b2Body_SetGravityScale(bodyId: b2BodyId, gravityScale: number): void;

/**
 * Gets the gravity scale of a body.
 * @param bodyId - The identifier of the body to query.
 * @returns The gravity scale factor of the body.
 */
declare function b2Body_GetGravityScale(bodyId: b2BodyId): number;

/**
 * Determines if a body is currently in the awake set by checking its set index
 * against the b2_awakeSet type. Bodies in the awake set are actively participating
 * in the simulation.
 * @param bodyId - The identifier for the body to check.
 * @returns True if the body is in the awake set, false otherwise.
 */
declare function b2Body_IsAwake(bodyId: b2BodyId): boolean;

/**
 * Controls whether a physics body is awake (active) or asleep (inactive).
 * When setting a body to sleep, it will split islands if there are pending constraint removals.
 * When waking a body, it will be moved from the sleeping set to the awake set.
 * @param bodyId - The ID of the body to modify
 * @param awake - True to wake the body, false to put it to sleep
 */
declare function b2Body_SetAwake(bodyId: b2BodyId, awake: boolean): void;

/**
 * Determines if a physics body is enabled by checking if it belongs to the
 * disabled set. A disabled body does not participate in collision detection
 * or dynamics simulation.
 * @param bodyId - The identifier for the body to check.
 * @returns True if the body is enabled, false if disabled.
 */
declare function b2Body_IsEnabled(bodyId: b2BodyId): boolean;

/**
 * Returns whether the specified body has sleep enabled. When sleep is enabled,
 * the body can automatically enter a sleep state when it becomes inactive.
 * @param bodyId - The identifier for the body to check.
 * @returns True if sleep is enabled for the body, false otherwise.
 */
declare function b2Body_IsSleepEnabled(bodyId: b2BodyId): boolean;

/**
 * Sets the minimum velocity threshold that determines when a body can transition to a sleeping state.
 * When a body's velocity falls below this threshold, it becomes eligible for sleeping.
 * @param bodyId - The identifier for the body to modify.
 * @param sleepVelocity - The velocity threshold below which the body can sleep.
 */
declare function b2Body_SetSleepThreshold(bodyId: b2BodyId, sleepVelocity: number): void;

/**
 * Returns the minimum speed threshold below which a body can be put to sleep
 * to optimize simulation performance.
 * @param bodyId - The identifier for the body.
 * @returns The sleep threshold value for the body.
 */
declare function b2Body_GetSleepThreshold(bodyId: b2BodyId): number;

/**
 * Enables or disables sleep capability for a specific body. When sleep is disabled,
 * the body will be woken if it was sleeping.
 * @param bodyId - The identifier for the body to modify
 * @param enableSleep - True to enable sleep capability, false to disable it
 */
declare function b2Body_EnableSleep(bodyId: b2BodyId, enableSleep: boolean): void;

/**
 * Disables a body in the physics simulation by removing it from its current solver set
 * and moving it to the disabled set. The function removes all contacts associated with
 * the body, removes it from any island it belongs to, destroys shape proxies in the
 * broad phase, and transfers associated joints to the disabled set.
 * @param bodyId - The identifier of the body to disable
 */
declare function b2Body_Disable(bodyId: b2BodyId): void;

/**
 * Enables a previously disabled body in the physics world. When enabled, the body's
 * shapes are added to the broad-phase collision system, and its joints are
 * reconnected to the simulation. The body is moved from the disabled set to either
 * the static set or awake set based on its type.
 * @param bodyId - The identifier of the body to enable
 */
declare function b2Body_Enable(bodyId: b2BodyId): void;

/**
 * Sets the fixed rotation state of a body. When enabled, the body will not rotate
 * and any existing angular velocity is cleared. The body's mass data is updated
 * to reflect the change in rotational constraints.
 * @param bodyId - The identifier for the body to modify
 * @param flag - True to fix the rotation, false to allow rotation
 */
declare function b2Body_SetFixedRotation(bodyId: b2BodyId, flag: boolean): void;

/**
 * Checks whether a body has fixed rotation enabled. When fixed rotation is enabled,
 * the body will not rotate in response to torques or collisions.
 * @param bodyId - The identifier for the body to check
 * @returns True if the body has fixed rotation enabled, false otherwise
 */
declare function b2Body_IsFixedRotation(bodyId: b2BodyId): boolean;

/**
 * Sets whether a body should be treated as a bullet for continuous collision detection.
 * Bullet bodies are designed for fast moving objects that require more precise
 * collision detection.
 * @param bodyId - The identifier for the physics body.
 * @param flag - True to enable bullet mode, false to disable it.
 */
declare function b2Body_SetBullet(bodyId: b2BodyId, flag: boolean): void;

/**
 * Retrieves the bullet status of a body from the physics simulation.
 * Bullet bodies undergo continuous collision detection for improved
 * accuracy with fast-moving objects.
 * @param bodyId - The identifier for the body to check.
 * @returns True if the body is a bullet, false otherwise.
 */
declare function b2Body_IsBullet(bodyId: b2BodyId): boolean;

/**
 * Enables or disables hit event detection for all shapes attached to a body.
 * @param bodyId - The identifier of the body to modify
 * @param enableHitEvents - Whether to enable or disable hit events
 */
declare function b2Body_EnableHitEvents(bodyId: b2BodyId, enableHitEvents: boolean): void;

/**
 * Returns the total count of shapes currently attached to the specified body
 * in the physics world.
 * @param bodyId - The identifier for the body to query.
 * @returns The number of shapes attached to the body.
 */
declare function b2Body_GetShapeCount(bodyId: b2BodyId): number;

/**
 * Retrieves all shapes attached to the specified body by traversing the linked list
 * of shapes starting from the body's headShapeId. Each shape ID is stored in the
 * provided shapeArray and the total count is returned.
 * If shapeArray is already large enough we can avoid the overhead of growing the array.
 * @param bodyId - The identifier of the body to get shapes from.
 * @param shapeArray - An array to store the retrieved shape IDs.
 * @returns The number of shapes found on the body.
 */
declare function b2Body_GetShapes(bodyId: b2BodyId, shapeArray: b2ShapeId[]): number;

/**
 * Returns the total count of joints that are connected to the specified body.
 * @param bodyId - The identifier for the body to query.
 * @returns The number of joints connected to the body.
 */
declare function b2Body_GetJointCount(bodyId: b2BodyId): number;

/**
 * Retrieves the IDs of all joints connected to the specified body, up to the given capacity.
 * The joint IDs are stored in the provided jointArray. The function traverses the body's
 * joint list and copies each joint's ID information including the index, world ID and revision.
 * @param bodyId - The ID of the body to get joints for
 * @param jointArray - Array to store the joint IDs
 * @param capacity - Maximum number of joints to retrieve
 * @returns The number of joints found and stored in jointArray
 */
declare function b2Body_GetJoints(bodyId: b2BodyId, jointArray: b2JointId[], capacity: number): number;

declare namespace Broadphase { }

/**
 * The broad-phase is used for computing pairs and performing volume queries and ray casts.
 * This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
 * It is up to the client to consume the new pairs and to track subsequent overlap.
 */
declare class b2BroadPhase {
}

declare namespace ConstraintGraph { }

declare namespace Contact { }

declare namespace ContactSolver { }

declare namespace Core { }

/**
 * Sets the global scaling factor that defines how many length units in the physics
 * simulation correspond to one meter in the real world. This affects all subsequent
 * physics calculations in the Box2D engine.
 * @param lengthUnits - The number of length units that represent one meter.
 */
declare function b2SetLengthUnitsPerMeter(lengthUnits: number): void;

/**
 * Returns the value of b2_lengthUnitsPerMeter, which defines the conversion factor
 * between physics simulation units and real-world meters.
 * @returns The number of length units per meter used in the physics simulation.
 */
declare function b2GetLengthUnitsPerMeter(): number;

/**
 * This function sets the global assertion handler used by Box2D for runtime checks.
 * The assertion handler is called when a Box2D assertion fails.
 * @param assertFcn - The assertion function to handle Box2D assertions.
 * If null, assertions will be disabled.
 */
declare function b2SetAssertFcn(assertFcn: ((...params: any[]) => any) | null): void;

/**
 * This function creates and returns a new b2Version object initialized with Box2D version 3.0.0
 * @returns A b2Version object containing major version 3, minor version 0, and revision 0
 */
declare function b2GetVersion(): b2Version;

declare namespace DebugDraw { }

/**
 * Creates a debug drawing interface for Box2D that renders shapes to a canvas context.
 * The canvas is automatically sized to 1280x720 or 720x1280 based on window orientation.
 * All coordinates are transformed from Box2D world space to screen space.
 * This feature isn't meant for production. It is purely for debugging and testing. The code
 * is not optimized, stable or extensible. Implement your own drawing code for production.
 * @param canvas - The canvas element to draw on
 * @param ctx - The 2D rendering context for the canvas
 * @param [scale = 20] - The scale factor to convert Box2D coordinates to pixels
 * @returns A debug draw instance with methods for rendering Box2D shapes
 * The debug draw instance includes methods for drawing:
 * - Polygons (outlined and filled)
 * - Circles (outlined and filled)
 * - Capsules (outlined and filled)
 * - Images mapped to shapes
 * - Line segments
 * - Points
 * - Transforms
 */
declare function CreateDebugDraw(canvas: HTMLCanvasElement, ctx: CanvasRenderingContext2D, scale?: number): b2DebugDraw;

/**
 * Creates an animation loop using requestAnimationFrame that tracks timing information
 * and FPS. The callback is invoked each frame with the time delta, total time, and
 * current FPS. Frame delta time is capped at 100ms to avoid large time steps.
 * @param callback - Function to call each frame with signature (deltaTime, totalTime, currentFps)
 * @param callback.deltaTime - Time elapsed since last frame in seconds, capped at 0.1s
 * @param callback.totalTime - Total accumulated time in seconds
 * @param callback.currentFps - Current frames per second, updated once per second
 */
declare function RAF(callback: {
    deltaTime: number;
    totalTime: number;
    currentFps: number;
}): void;

/**
 * IMAGE HELPERS
 */
declare function loadPNGImage(): void;

/**
 * Attaches an image to the last shape of a Box2D body. The function loads a PNG image
 * asynchronously and sets up drawing parameters including offset, scale, and source
 * rectangle coordinates. The image is stored in the shape's properties for later rendering.
 * @param worldId - The ID of the Box2D world
 * @param bodyId - The ID of the body to attach the image to
 * @param path - Directory path where the image is located
 * @param imgName - Name of the image file
 * @param [drawOffset = null] - Offset vector for drawing the image
 * @param [drawScale = null] - Scale vector for drawing the image
 * @param [sourcePosition = null] - Position in the source image to start drawing from
 * @param [sourceSize = null] - Size of the region to draw from the source image
 * @returns The modified shape object with attached image properties
 */
declare function AttachImage(worldId: number, bodyId: number, path: string, imgName: string, drawOffset?: b2Vec2, drawScale?: b2Vec2, sourcePosition?: b2Vec2, sourceSize?: b2Vec2): any;

/**
 * UI HELPERS
 */
declare function getMousePosUV(): void;

/**
 * Converts screen/canvas coordinates to world space coordinates in the Box2D physics system.
 * @example
 * // Convert mouse click position to world coordinates
 * const worldPos = ConvertScreenToWorld(myCanvas, 30, mousePos);
 * @param canvas - The canvas element being used for rendering
 * @param drawScale - The scale factor between screen and world coordinates
 * @param ps - The screen position coordinates
 * @returns A vector containing the world space coordinates
 */
declare function ConvertScreenToWorld(canvas: HTMLCanvasElement, drawScale: number, ps: any): b2Vec2;

/**
 * Transforms a position from world space to screen space, taking into account
 * the canvas dimensions, aspect ratio, and zoom level. The function maps the
 * world coordinates to normalized coordinates (0-1) and then scales them to
 * screen pixels.
 * @param canvas - The canvas element used for rendering
 * @param drawScale - The scale factor for converting world units to pixels
 * @param pw - The world position to convert
 * @returns The converted screen coordinates as a b2Vec2
 */
declare function ConvertWorldToScreen(canvas: HTMLCanvasElement, drawScale: number, pw: b2Vec2): b2Vec2;

declare namespace Distance { }

/**
 * Calculates an intermediate transform by linearly interpolating between two states
 * defined in a sweep motion. The resulting transform accounts for both translation
 * and rotation, adjusted by the local center offset.
 * @param sweep - A sweep object containing initial (c1, q1) and final (c2, q2)
 * positions and rotations, along with a local center offset.
 * @param time - Interpolation factor between 0 and 1, where 0 represents the initial
 * state and 1 represents the final state.
 * @returns A transform object containing the interpolated position (p) and
 * rotation (q) at the specified time.
 */
declare function b2GetSweepTransform(sweep: b2Sweep, time: number): b2Transform;

/**
 * Calculates the minimum distance between two line segments defined by their endpoints.
 * @param p1X - X coordinate of the first point of segment 1
 * @param p1Y - Y coordinate of the first point of segment 1
 * @param q1X - X coordinate of the second point of segment 1
 * @param q1Y - Y coordinate of the second point of segment 1
 * @param p2X - X coordinate of the first point of segment 2
 * @param p2Y - Y coordinate of the first point of segment 2
 * @param q2X - X coordinate of the second point of segment 2
 * @param q2Y - Y coordinate of the second point of segment 2
 * @returns An object containing:
 * - fraction1: {number} Parameter along segment 1 for closest point (0-1)
 * - fraction2: {number} Parameter along segment 2 for closest point (0-1)
 * - distanceSquared: {number} Square of the minimum distance between segments
 */
declare function b2SegmentDistance(p1X: number, p1Y: number, q1X: number, q1Y: number, p2X: number, p2Y: number, q2X: number, q2Y: number): b2SegmentDistanceResult;

/**
 * @param vertices - Array of 2D vectors representing the vertices
 * @param count - Number of vertices to process (max B2_MAX_POLYGON_VERTICES)
 * @param radius - Radius value for the proxy
 * @returns A new distance proxy containing the processed vertices
 */
declare function b2MakeProxy(vertices: b2Vec2[], count: number, radius: number): b2DistanceProxy;

/**
 * Computes the distance between two convex shapes using the GJK (Gilbert-Johnson-Keerthi) algorithm.
 * @param cache - Cache object to store and retrieve simplex data between calls
 * @param input - Input parameters containing:
 * - proxyA: First shape proxy
 * - proxyB: Second shape proxy
 * - transformA: Transform for first shape
 * - transformB: Transform for second shape
 * - useRadii: Boolean flag for including shape radii in calculation
 * @param simplexes - Optional array to store simplex history
 * @param simplexCapacity - Maximum number of simplexes to store
 * @returns Output containing:
 * - pointA: Closest point on shape A
 * - pointB: Closest point on shape B
 * - distance: Distance between the shapes
 * - iterations: Number of iterations performed
 * - simplexCount: Number of simplexes stored
 */
declare function b2ShapeDistance(cache: b2DistanceCache, input: b2DistanceInput, simplexes: b2Simplex[], simplexCapacity: number): b2DistanceOutput;

/**
 * Uses an iterative algorithm to determine if and when two convex shapes will collide
 * during a linear motion. Shape B is translated while shape A remains stationary.
 * The algorithm uses support points and simplexes to determine the closest points
 * between the shapes.
 * @param input - Contains:
 * proxyA - First shape proxy
 * proxyB - Second shape proxy
 * transformA - Transform of first shape
 * transformB - Transform of second shape
 * translationB - Translation vector for second shape
 * maxFraction - Maximum fraction of motion to check
 * @returns Contains:
 * fraction - Time of impact fraction [0,maxFraction]
 * point - Point of impact
 * normal - Surface normal at impact
 * iterations - Number of iterations used
 * hit - Whether a collision was detected
 */
declare function b2ShapeCast(input: b2ShapeCastPairInput): b2CastOutput;

/**
 * Computes when two moving shapes first collide during their motion sweeps.
 * Uses conservative advancement and binary search to find the first time of impact
 * or determine that no impact occurs during the time interval.
 * @param input - Input parameters containing:
 * - proxyA: First shape proxy
 * - proxyB: Second shape proxy
 * - sweepA: Motion sweep for first shape
 * - sweepB: Motion sweep for second shape
 * - tMax: Maximum time interval
 * @returns Output containing:
 * - state: The termination state (Unknown, Failed, Overlapped, Hit, Separated)
 * - t: Time of impact (between 0 and tMax)
 */
declare function b2TimeOfImpact(input: b2TOIInput): b2TOIOutput;

declare namespace DistanceJoint { }

/**
 * Sets the target length of a distance joint. The length value is automatically
 * clamped between b2_linearSlop and B2_HUGE. After setting the length,
 * the joint's impulse values are reset to zero.
 * @param jointId - The identifier of the distance joint to modify.
 * @param length - The desired length of the joint, clamped between b2_linearSlop and B2_HUGE.
 */
declare function b2DistanceJoint_SetLength(jointId: b2JointId, length: number): void;

/**
 * Returns the current length of a distance joint. The joint must be of type b2_distanceJoint.
 * @param jointId - The identifier for the distance joint.
 * @returns The current length of the distance joint.
 */
declare function b2DistanceJoint_GetLength(jointId: b2JointId): number;

/**
 * Sets the enable/disable state of the limit constraint for a distance joint.
 * The joint must be of type b2_distanceJoint or an error will occur.
 * @param jointId - The identifier for the distance joint to modify.
 * @param enableLimit - True to enable the joint's limit, false to disable it.
 */
declare function b2DistanceJoint_EnableLimit(jointId: b2JointId, enableLimit: boolean): void;

/**
 * @param jointId - The identifier for the distance joint to check.
 * @returns True if the limit is enabled, false otherwise.
 */
declare function b2DistanceJoint_IsLimitEnabled(jointId: b2JointId): boolean;

/**
 * Sets the minimum and maximum length constraints for a distance joint.
 * The values are clamped between b2_linearSlop and B2_HUGE.
 * The function resets all impulse values to zero after updating the length range.
 * @param jointId - The identifier for the distance joint
 * @param minLength - The minimum allowed length of the joint
 * @param maxLength - The maximum allowed length of the joint
 */
declare function b2DistanceJoint_SetLengthRange(jointId: b2JointId, minLength: number, maxLength: number): void;

/**
 * Gets the minimum length of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The minimum length value of the distance joint.
 */
declare function b2DistanceJoint_GetMinLength(jointId: b2JointId): number;

/**
 * Gets the maximum length of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The maximum length value of the distance joint.
 */
declare function b2DistanceJoint_GetMaxLength(jointId: b2JointId): number;

/**
 * Calculates the current distance between the two anchor points of a distance joint
 * in world coordinates. The function transforms the local anchor points of both bodies
 * to world coordinates and computes the distance between them.
 * @param jointId - The identifier for the distance joint
 * @returns The current length between the two anchor points of the distance joint
 */
declare function b2DistanceJoint_GetCurrentLength(jointId: b2JointId): number;

/**
 * Sets the spring behavior state of a distance joint. When enabled, the joint acts like
 * a spring between two bodies. When disabled, the joint maintains a fixed distance
 * between the connected bodies.
 * @param jointId - The identifier of the distance joint to modify.
 * @param enableSpring - True to enable spring behavior, false to disable it.
 */
declare function b2DistanceJoint_EnableSpring(jointId: b2JointId, enableSpring: boolean): void;

/**
 * Returns the state of the spring enable flag for the specified distance joint.
 * The function validates that the joint is of type b2_distanceJoint before
 * accessing the spring enable property.
 * @param jointId - The identifier for the distance joint to check.
 * @returns True if the spring mechanism is enabled, false otherwise.
 */
declare function b2DistanceJoint_IsSpringEnabled(jointId: b2JointId): boolean;

/**
 * Sets the spring oscillation frequency for a distance joint. The frequency
 * determines how quickly the spring oscillates when disturbed from equilibrium.
 * @param jointId - The identifier for the distance joint to modify.
 * @param hertz - The spring frequency in Hertz (oscillations per second).
 */
declare function b2DistanceJoint_SetSpringHertz(jointId: b2JointId, hertz: number): void;

/**
 * Sets the damping ratio parameter for a distance joint's spring mechanism.
 * The joint must be of type b2_distanceJoint.
 * @param jointId - The identifier for the distance joint to modify
 * @param dampingRatio - The damping ratio for the spring (0 = no damping, 1 = critical damping)
 */
declare function b2DistanceJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the hertz frequency parameter of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The hertz frequency value of the distance joint.
 */
declare function b2DistanceJoint_GetHertz(jointId: number): number;

/**
 * Gets the damping ratio of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The damping ratio of the distance joint.
 */
declare function b2DistanceJoint_GetDampingRatio(jointId: number): number;

/**
 * Enables or disables the motor on a distance joint. When the motor state changes,
 * the motor impulse is reset to zero.
 * @param jointId - The identifier for the distance joint
 * @param enableMotor - True to enable the motor, false to disable it
 */
declare function b2DistanceJoint_EnableMotor(jointId: b2JointId, enableMotor: boolean): void;

/**
 * @param jointId - The identifier for the distance joint to check.
 * @returns True if the motor is enabled, false otherwise.
 */
declare function b2DistanceJoint_IsMotorEnabled(jointId: b2JointId): boolean;

/**
 * Sets the motor speed for a distance joint.
 * @param jointId - The identifier for the distance joint to modify.
 * @param motorSpeed - The new motor speed value to set.
 */
declare function b2DistanceJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: number): void;

/**
 * Gets the motor speed of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The current motor speed of the distance joint in radians per second.
 */
declare function b2DistanceJoint_GetMotorSpeed(jointId: b2JointId): number;

/**
 * Calculates the motor force by dividing the joint's motor impulse by the inverse time step (inv_h).
 * The joint must be of type b2_distanceJoint.
 * @param jointId - The identifier for the distance joint.
 * @returns The current motor force in Newtons, calculated as the motor impulse divided by the step time.
 */
declare function b2DistanceJoint_GetMotorForce(jointId: b2JointId): number;

/**
 * Sets the maximum motor force for a distance joint.
 * @param jointId - The identifier for the distance joint to modify.
 * @param force - The maximum force the motor can generate.
 */
declare function b2DistanceJoint_SetMaxMotorForce(jointId: b2JointId, force: number): void;

/**
 * Gets the maximum motor force of a distance joint.
 * @param jointId - The identifier for the distance joint.
 * @returns The maximum force that can be applied by the joint's motor.
 */
declare function b2DistanceJoint_GetMaxMotorForce(jointId: b2JointId): number;

declare namespace DynamicTree { }

/**
 * Creates a b2DynamicTree data structure used for efficient spatial partitioning.
 * The tree is initialized with a pre-allocated pool of nodes linked in a free list.
 * @returns A new dynamic tree with:
 * - Initial node capacity of 16
 * - Empty root (B2_NULL_INDEX)
 * - Initialized node array with parent/next pointers
 * - All nodes set to height -1
 * - Free list starting at index 0
 * - Zero proxy count
 * - Null leaf indices and centers
 * - Zero rebuild capacity
 */
declare function b2DynamicTree_Create(): b2DynamicTree;

/**
 * Clears the nodes, leaf indices, and leaf centers arrays of the dynamic tree,
 * effectively destroying the tree's data structure.
 * @param tree - The dynamic tree to destroy.
 */
declare function b2DynamicTree_Destroy(tree: b2DynamicTree): void;

/**
 * Creates a proxy in a dynamic tree for collision detection. The proxy is added as a leaf node.
 * @param tree - The dynamic tree to add the proxy to
 * @param aabb - The axis-aligned bounding box for the proxy
 * @param categoryBits - The collision category bits for filtering
 * @param userData - User data associated with this proxy
 * @returns The ID of the created proxy node
 */
declare function b2DynamicTree_CreateProxy(tree: b2DynamicTree, aabb: b2AABB, categoryBits: number, userData: number): number;

/**
 * Removes a leaf node from the tree, frees the node's memory, and decrements
 * the proxy count. The proxy must be a valid leaf node in the tree.
 * @param tree - The dynamic tree containing the proxy
 * @param proxyId - The ID of the proxy to destroy (must be >= 0 and < nodeCapacity)
 */
declare function b2DynamicTree_DestroyProxy(tree: b2DynamicTree, proxyId: number): void;

/**
 * Returns the number of proxies currently stored in the dynamic tree by accessing
 * the proxyCount property.
 * @param tree - The dynamic tree to query.
 * @returns The total count of proxies in the tree.
 */
declare function b2DynamicTree_GetProxyCount(tree: b2DynamicTree): number;

/**
 * Updates the position of a proxy in the dynamic tree by removing and reinserting it
 * with a new AABB.
 * @param tree - The dynamic tree containing the proxy
 * @param proxyId - The ID of the proxy to move (must be within tree.nodeCapacity)
 * @param aabb - The new axis-aligned bounding box for the proxy
 */
declare function b2DynamicTree_MoveProxy(tree: b2DynamicTree, proxyId: number, aabb: b2AABB): void;

/**
 * Updates a proxy's AABB in the dynamic tree and rebalances the tree structure by
 * enlarging parent nodes' AABBs as needed.
 * @param tree - The dynamic tree containing the proxy
 * @param proxyId - The ID of the proxy to enlarge
 * @param aabb - The new axis-aligned bounding box for the proxy
 */
declare function b2DynamicTree_EnlargeProxy(tree: b2DynamicTree, proxyId: number, aabb: b2AABB): void;

/**
 * Returns the height of the specified dynamic tree by accessing the height property
 * of the root node. The height represents the maximum number of levels from the root
 * to any leaf node.
 * @param tree - The dynamic tree to measure
 * @returns The height of the tree. Returns 0 if the tree is empty (root is null)
 */
declare function b2DynamicTree_GetHeight(tree: b2DynamicTree): number;

/**
 * Computes the sum of all internal node perimeters (excluding leaves and root)
 * divided by the root node perimeter. This ratio provides a measure of the tree's
 * spatial organization.
 * @param tree - The dynamic tree structure to analyze
 * @returns The ratio of total internal node perimeter to root perimeter. Returns 0 if the tree is empty.
 */
declare function b2DynamicTree_GetAreaRatio(tree: b2DynamicTree): number;

/**
 * Performs validation checks on a b2DynamicTree data structure to ensure
 * its internal state is consistent. This is typically used for debugging
 * and testing purposes.
 * @param tree - The dynamic tree to validate
 */
declare function b2DynamicTree_Validate(tree: b2DynamicTree): void;

/**
 * Iterates through all non-leaf nodes in the tree and calculates the absolute height difference
 * between their child nodes. Returns the largest height difference found.
 * @param tree - The dynamic tree to analyze
 * @returns The maximum balance value (height difference) found between any pair of sibling nodes
 */
declare function b2DynamicTree_GetMaxBalance(tree: b2DynamicTree): number;

/**
 * Rebuilds a dynamic tree from the bottom up by iteratively combining nodes with the lowest perimeter cost.
 * The function first identifies all leaf nodes, then pairs them based on minimum combined AABB perimeter,
 * creating parent nodes until a single root node remains.
 * @param tree - The dynamic tree to rebuild
 */
declare function b2DynamicTree_RebuildBottomUp(tree: b2DynamicTree): void;

/**
 * Updates the axis-aligned bounding boxes (AABBs) of all nodes in the tree by
 * subtracting the newOrigin vector from both their lower and upper bounds.
 * @param tree - The dynamic tree whose nodes will be shifted
 * @param newOrigin - The vector to subtract from all node boundaries
 */
declare function b2DynamicTree_ShiftOrigin(tree: b2DynamicTree, newOrigin: b2Vec2): void;

/**
 * Calculates the memory footprint by summing:
 * - Base object properties
 * - Node array storage
 * - Rebuild capacity storage
 * @param tree - The dynamic tree instance to measure.
 * @returns The estimated memory usage in bytes.
 */
declare function b2DynamicTree_GetByteCount(tree: b2DynamicTree): number;

/**
 * Queries a dynamic tree to find all nodes that overlap with the given AABB and match the category mask bits.
 * Uses a stack-based traversal to efficiently search the tree structure.
 * @param tree - The dynamic tree to query
 * @param aabb - The axis-aligned bounding box to test for overlaps
 * @param maskBits - Category bits used to filter nodes
 * @param callback - Function called for each overlapping leaf node.
 * Return false to terminate early, true to continue.
 * @param context - User context data passed to the callback function
 */
declare function b2DynamicTree_Query(tree: b2DynamicTree, aabb: b2AABB, maskBits: number, callback: (...params: any[]) => any, context: any): void;

/**
 * Traverses the dynamic tree and finds all leaf nodes that intersect with the input ray.
 * For each intersection, calls the callback function which can control continuation of the search.
 * Uses an AABB overlap test and separating axis test to efficiently cull branches.
 * @param tree - The dynamic tree to query
 * @param input - Input parameters for the ray cast including:
 * - origin: b2Vec2 starting point
 * - translation: b2Vec2 ray direction and length
 * - maxFraction: number maximum ray length multiplier
 * @param maskBits - Bit mask to filter nodes by category
 * @param callback - Function called for each leaf node intersection
 * - Parameters: (input: b2RayCastInput, nodeId: number, userData: any, context: any)
 * - Returns: number between 0 and 1 to continue search, 0 to terminate
 * @param context - User context passed to callback
 */
declare function b2DynamicTree_RayCast(tree: b2DynamicTree, input: b2RayCastInput, maskBits: number, callback: (...params: any[]) => any, context: any): void;

/**
 * Performs a shape cast query against nodes in a dynamic tree, testing for overlaps
 * between a moving shape and static shapes in the tree.
 * @param tree - The dynamic tree to query against
 * @param input - Contains the shape definition, translation vector,
 * radius, and maximum fraction for the cast
 * @param maskBits - Bit mask to filter tree nodes by category
 * @param callback - Function called when potential overlaps are found.
 * Returns a fraction value to continue or terminate the cast
 * @param context - User data passed to the callback function
 * @returns The callback function should have the signature:
 * function(input: b2ShapeCastInput, nodeId: number, userData: any, context: any): number
 * It should return 0 to terminate the cast, or a fraction between 0 and 1 to update the cast distance
 */
declare function b2DynamicTree_ShapeCast(tree: b2DynamicTree, input: b2ShapeCastInput, maskBits: number, callback: (...params: any[]) => any, context: any): void;

/**
 * Rebuilds a dynamic tree by collecting all leaf nodes and reconstructing the tree structure.
 * The function deallocates internal nodes during traversal and builds a new balanced tree
 * from the collected leaf nodes.
 * @param tree - The dynamic tree to rebuild
 * @returns The number of leaf nodes in the rebuilt tree
 */
declare function b2DynamicTree_Rebuild(tree: b2DynamicTree): number;

declare namespace Geometry { }

/**
 * Checks if a ray cast input is valid by verifying:
 * - The origin vector is valid
 * - The translation vector is valid
 * - The maxFraction is a valid number
 * - The maxFraction is between 0 and B2_HUGE(exclusive)
 * @param input - The ray cast input to validate, containing:
 * - origin: b2Vec2 - Starting point of the ray
 * - translation: b2Vec2 - Direction and length of the ray
 * - maxFraction: number - Maximum fraction of translation to check
 * @returns True if the ray cast input is valid, false otherwise.
 */
declare function b2IsValidRay(input: b2RayCastInput): boolean;

/**
 * Creates a b2Polygon from a convex hull. If the hull has fewer than 3 points, returns a
 * square shape. The function computes the polygon's vertices, edge normals, and centroid.
 * Each edge normal is a unit vector perpendicular to the corresponding edge.
 * @param hull - A convex hull structure containing points that define the polygon vertices
 * @param radius - The radius used to round the corners of the polygon
 * @param [forceCheck = true] - Whether to enforce hull validation
 * @returns A new polygon shape with computed vertices, normals, and centroid
 */
declare function b2MakePolygon(hull: b2Hull, radius: number, forceCheck?: boolean): b2Polygon;

/**
 * Creates a polygon shape from a hull with specified radius and transform
 * @param hull - The input hull to create the polygon from
 * @param radius - The radius to offset the polygon vertices
 * @param transform - Transform to apply to the hull points
 * @param [forceCheck = true] - Whether to force validation check of the hull
 * @returns A new polygon shape with transformed vertices, computed normals and centroid
 */
declare function b2MakeOffsetPolygon(hull: b2Hull, radius: number, transform: b2Transform, forceCheck?: boolean): b2Polygon;

/**
 * Creates a square polygon by calling b2MakeBox with equal dimensions.
 * The square is centered at the origin with sides of length 2h.
 * @param h - The half-width and half-height of the square.
 * @returns A polygon object representing a square centered at the origin.
 */
declare function b2MakeSquare(h: number): b2Polygon;

/**
 * Creates a rectangular polygon shape centered at the origin with specified half-widths.
 * The vertices are arranged counter-clockwise starting from the bottom-left corner.
 * The shape includes pre-computed normals for each edge.
 * @param hx - Half-width of the box in the x-direction (must be positive)
 * @param hy - Half-height of the box in the y-direction (must be positive)
 * @returns A polygon shape representing a rectangle with:
 * - 4 vertices at (-hx,-hy), (hx,-hy), (hx,hy), (-hx,hy)
 * - 4 normals pointing outward from each edge
 * - radius of 0
 * - centroid at (0,0)
 */
declare function b2MakeBox(hx: number, hy: number): b2Polygon;

/**
 * Creates a rounded box shape by generating a box with specified dimensions and corner radius.
 * @param hx - Half-width of the box along the x-axis
 * @param hy - Half-height of the box along the y-axis
 * @param radius - Radius of the rounded corners
 * @returns A polygon shape representing a rounded box
 */
declare function b2MakeRoundedBox(hx: number, hy: number, radius: number): b2Polygon;

/**
 * Creates a b2Polygon representing a rectangle with the given dimensions. The box is centered
 * at the specified position and rotated by the given angle. The resulting polygon includes
 * 4 vertices, 4 normals, and has its centroid set to the center position.
 * @param hx - Half-width of the box along the x-axis
 * @param hy - Half-height of the box along the y-axis
 * @param center - The center position of the box
 * @param angle - The rotation angle of the box in radians
 * @returns A polygon shape representing the box with 4 vertices and normals
 */
declare function b2MakeOffsetBox(hx: number, hy: number, center: b2Vec2, angle: number): b2Polygon;

/**
 * Applies a rigid body transformation to a polygon by:
 * 1. Transforming each vertex using the full transform
 * 2. Rotating each normal vector using only the rotation component
 * 3. Transforming the centroid using the full transform
 * @param transform - The transformation to apply, consisting of a position vector and rotation.
 * @param polygon - The polygon to transform, containing vertices, normals and centroid.
 * @returns The transformed polygon with updated vertices, normals and centroid.
 */
declare function b2TransformPolygon(transform: b2Transform, polygon: b2Polygon): b2Polygon;

/**
 * Calculates the mass, center of mass, and rotational inertia for a circle shape
 * with given density. The rotational inertia is computed about the center of mass
 * using the parallel axis theorem when the circle's center is offset from the origin.
 * @param shape - A circle shape object containing radius and center properties
 * @param density - The density of the circle in mass per unit area
 * @returns An object containing:
 * - mass: The total mass of the circle
 * - center: The center of mass (copied from shape.center)
 * - rotationalInertia: The rotational inertia about the center of mass
 */
declare function b2ComputeCircleMass(shape: b2Circle, density: number): b2MassData;

/**
 * Computes mass properties for a capsule shape, including total mass, center of mass,
 * and rotational inertia. A capsule consists of a rectangle with semicircles at both ends.
 * @param shape - A capsule shape defined by two centers (center1, center2) and a radius
 * @param density - The density of the capsule in mass per unit area
 * @returns An object containing:
 * - mass: The total mass of the capsule
 * - center: A b2Vec2 representing the center of mass
 * - rotationalInertia: The moment of inertia about the center of mass
 */
declare function b2ComputeCapsuleMass(shape: b2Capsule, density: number): b2MassData;

/**
 * Computes mass properties for a polygon shape, including mass, center of mass, and rotational inertia.
 * Handles special cases for 1-vertex (circle) and 2-vertex (capsule) polygons.
 * For polygons with 3 or more vertices, calculates properties using triangulation.
 * @param shape - The polygon shape containing vertices, normals, count, and radius
 * @param density - The density of the shape in mass per unit area
 * @returns Object containing:
 * - mass: Total mass of the shape
 * - center: Center of mass as b2Vec2
 * - rotationalInertia: Moment of inertia about the center of mass
 */
declare function b2ComputePolygonMass(shape: b2Polygon, density: number): b2MassData;

/**
 * Calculates the AABB by transforming the circle's center point using the provided transform
 * and extending the bounds by the circle's radius in each direction.
 * @param shape - The circle shape containing center point and radius.
 * @param xf2 - The transform to be applied, consisting of a position (p) and rotation (q).
 * @returns An AABB object defined by minimum and maximum points that bound the transformed circle.
 */
declare function b2ComputeCircleAABB(shape: b2Circle, xf2: b2Transform): b2AABB;

/**
 * Calculates the minimum and maximum bounds of a capsule after applying a transform.
 * The AABB is computed by transforming the capsule's center points and extending
 * the bounds by the capsule's radius in all directions.
 * @param shape - A capsule shape defined by two centers and a radius.
 * @param xf2 - A transform containing position and rotation to be applied to the capsule.
 * @returns An AABB that encompasses the transformed capsule shape.
 */
declare function b2ComputeCapsuleAABB(shape: b2Capsule, xf2: b2Transform): b2AABB;

/**
 * Computes the Axis-Aligned Bounding Box (AABB) for a polygon shape after applying a transform.
 * The AABB includes the polygon's radius in its calculations.
 * @param shape - The polygon shape containing vertices and radius
 * @param xf2 - The transform to apply, consisting of position (p) and rotation (q)
 * @returns An AABB object with lower and upper bounds that encompass the transformed polygon
 */
declare function b2ComputePolygonAABB(shape: b2Polygon, xf2: b2Transform): b2AABB;

/**
 * Transforms the segment's endpoints using the provided transform, then creates an AABB
 * that encompasses the transformed segment by finding the minimum and maximum coordinates
 * of the transformed endpoints.
 * @param shape - A line segment defined by two points (point1 and point2)
 * @param xf2 - A transform containing position and rotation to be applied to the segment
 * @returns An AABB that contains the transformed line segment
 */
declare function b2ComputeSegmentAABB(shape: b2Segment, xf2: b2Transform): b2AABB;

/**
 * Tests if a point lies within a circle by comparing the squared distance between
 * the point and circle's center against the circle's squared radius.
 * @param point - The point to test, represented as a 2D vector.
 * @param shape - The circle to test against, containing center and radius properties.
 * @returns True if the point lies within or on the circle's boundary, false otherwise.
 */
declare function b2PointInCircle(point: b2Vec2, shape: b2Circle): boolean;

/**
 * A capsule is defined by two end centers (center1 and center2) and a radius.
 * The function calculates if the given point lies within the capsule by:
 * 1. Testing if the capsule has zero length (centers are identical)
 * 2. If not, finding the closest point on the line segment between centers
 * 3. Checking if the distance from the test point to the closest point is within the radius
 * @param point - The point to test
 * @param shape - The capsule shape defined by two centers and a radius
 * @returns True if the point lies inside or on the capsule, false otherwise
 */
declare function b2PointInCapsule(point: b2Vec2, shape: b2Capsule): boolean;

/**
 * Tests if a point lies inside a polygon shape by calculating the minimum distance
 * between the point and the polygon.
 * @param point - The point to test
 * @param shape - The polygon shape to test against
 * @returns True if the point is inside or on the polygon (within shape.radius), false otherwise
 */
declare function b2PointInPolygon(point: b2Vec2, shape: b2Polygon): boolean;

/**
 * Calculates the intersection point between a ray and a circle shape.
 * Returns early with no hit if the ray length is 0 or if the ray passes outside the circle radius.
 * @param input - The ray cast input parameters containing:
 * - origin: b2Vec2 starting point of the ray
 * - translation: b2Vec2 direction and length of the ray
 * - maxFraction: number maximum intersection distance as a fraction of ray length
 * @param shape - The circle shape to test against, containing:
 * - center: b2Vec2 position of circle center
 * - radius: number radius of the circle
 * @returns The ray cast results containing:
 * - hit: boolean whether the ray intersects the circle
 * - point: b2Vec2 point of intersection if hit is true
 * - normal: b2Vec2 surface normal at intersection point if hit is true
 * - fraction: number intersection distance as a fraction of ray length if hit is true
 */
declare function b2RayCastCircle(input: b2RayCastInput, shape: b2Circle): b2CastOutput;

/**
 * Performs a ray cast against a capsule shape. A capsule is defined by two centers and a radius.
 * If the capsule length is near zero, it degrades to a circle ray cast.
 * @param input - Contains ray cast parameters including:
 * - origin: b2Vec2 starting point of the ray
 * - translation: b2Vec2 direction and length of the ray
 * - maxFraction: number maximum intersection distance as a fraction of ray length
 * @param shape - The capsule to test against, containing:
 * - center1: b2Vec2 first endpoint of capsule centerline
 * - center2: b2Vec2 second endpoint of capsule centerline
 * - radius: number radius of the capsule
 * @returns Contains the ray cast results:
 * - fraction: number intersection distance as a fraction of ray length
 * - point: b2Vec2 point of intersection
 * - normal: b2Vec2 surface normal at intersection
 * - hit: boolean whether an intersection occurred
 */
declare function b2RayCastCapsule(input: b2RayCastInput, shape: b2Capsule): b2CastOutput;

/**
 * Performs a ray cast against a line segment, determining if and where the ray intersects the segment.
 * For one-sided segments, intersections are only detected from one side based on a cross product test.
 * @param input - Contains origin point, translation vector, and max fraction for the ray
 * @param shape - The line segment defined by two points (point1 and point2)
 * @param oneSided - Whether to treat the segment as one-sided
 * @returns Contains hit status, intersection point, normal, and fraction along the ray
 */
declare function b2RayCastSegment(input: b2RayCastInput, shape: b2Segment, oneSided: boolean): b2CastOutput;

/**
 * Performs a ray cast against a polygon shape, detecting intersection points and normals.
 * For non-zero radius polygons, converts the ray cast into a shape cast operation.
 * @param input - Contains origin point, translation vector, and max fraction
 * @param shape - The polygon to test against, containing vertices, normals, count and radius
 * @returns Contains:
 * - hit: boolean indicating if intersection occurred
 * - point: intersection point (if hit is true)
 * - normal: surface normal at intersection (if hit is true)
 * - fraction: fraction of translation where intersection occurred (if hit is true)
 */
declare function b2RayCastPolygon(input: b2RayCastInput, shape: b2Polygon): b2CastOutput;

/**
 * Performs shape casting between a circle and a set of points with radius.
 * @param input - Contains points array, count, radius, translation and maxFraction
 * @param shape - Circle shape with center point and radius
 * @returns The shape cast result containing intersection details
 */
declare function b2ShapeCastCircle(input: b2ShapeCastInput, shape: b2Circle): b2CastOutput;

/**
 * Performs shape casting for a capsule shape against a set of points.
 * @param input - Contains the target points, count, radius,
 * translation, and maxFraction for the shape cast
 * @param shape - The capsule shape defined by two centers and a radius
 * @returns The result of the shape cast operation
 */
declare function b2ShapeCastCapsule(input: b2ShapeCastInput, shape: b2Capsule): b2CastOutput;

/**
 * Performs a shape cast operation between a segment and another shape. The function creates
 * proxies for both shapes, sets up their initial transforms, and performs the cast operation
 * using the specified translation and maximum fraction.
 * @param input - Contains shape points, count, radius, translation, and maxFraction
 * @param shape - A segment defined by two points (point1 and point2)
 * @returns The result of the shape cast operation
 */
declare function b2ShapeCastSegment(input: b2ShapeCastInput, shape: b2Segment): b2CastOutput;

/**
 * Performs shape casting between a polygon shape and a set of points, checking for collisions
 * along a translation path.
 * @param input - Contains the points, count, radius, translation, and maxFraction for the cast
 * @param shape - The polygon shape to test against, containing vertices, count, and radius
 * @returns The result of the shape cast operation
 */
declare function b2ShapeCastPolygon(input: b2ShapeCastInput, shape: b2Polygon): b2CastOutput;

declare namespace Hull { }

/**
 * Computes the convex hull of a set of 2D points. The function:
 * - Filters duplicate points within a tolerance
 * - Finds extreme points to establish initial hull edges
 * - Recursively adds points to build the complete hull
 * - Removes collinear/near-collinear points from final hull
 * @param points - Array of 2D points to compute hull from
 * @param count - Number of points in the array
 * @returns A hull object containing the computed convex hull vertices
 */
declare function b2ComputeHull(points: b2Vec2[], count: number): b2Hull;

/**
 * Validates that a hull meets the requirements for a valid convex polygon:
 * - Has between 3 and B2_MAX_POLYGON_VERTICES points
 * - Points are in counter-clockwise order
 * - All points are behind the edges
 * - No collinear points within b2_linearSlop tolerance
 * @param hull - The hull to validate, containing points array and count
 * @returns True if the hull is valid, false otherwise
 */
declare function b2ValidateHull(hull: b2Hull): boolean;

declare namespace IDPool { }

/**
 * @property major - Significant changes
 * @property minor - Incremental changes
 * @property revision - Bug fixes
 */
declare class b2Version {
    /**
     * Significant changes
    */
    major: number;
    /**
     * Incremental changes
    */
    minor: number;
    /**
     * Bug fixes
    */
    revision: number;
}

/**
 * @property origin - Start point of the ray cast
 * @property translation - Translation of the ray cast
 * @property maxFraction - The maximum fraction of the translation to consider, typically 1
 */
declare class b2RayCastInput {
    /**
     * Start point of the ray cast
    */
    origin: b2Vec2;
    /**
     * Translation of the ray cast
    */
    translation: b2Vec2;
    /**
     * The maximum fraction of the translation to consider, typically 1
    */
    maxFraction: number;
}

/**
 * @property points - A point cloud to cast
 * @property count - The number of points
 * @property radius - The radius around the point cloud
 * @property translation - The translation of the shape cast
 * @property maxFraction - The maximum fraction of the translation to consider, typically 1
 */
declare class b2ShapeCastInput {
    /**
     * A point cloud to cast
    */
    points: b2Vec2[];
    /**
     * The number of points
    */
    count: number;
    /**
     * The radius around the point cloud
    */
    radius: number;
    /**
     * The translation of the shape cast
    */
    translation: b2Vec2;
    /**
     * The maximum fraction of the translation to consider, typically 1
    */
    maxFraction: number;
}

/**
 * @property normal - The surface normal at the hit point
 * @property point - The surface hit point
 * @property fraction - The fraction of the input translation at collision
 * @property iterations - The number of iterations used
 * @property hit - Did the cast hit?
 */
declare class b2CastOutput {
    /**
     * The surface normal at the hit point
    */
    normal: b2Vec2;
    /**
     * The surface hit point
    */
    point: b2Vec2;
    /**
     * The fraction of the input translation at collision
    */
    fraction: number;
    /**
     * The number of iterations used
    */
    iterations: number;
    /**
     * Did the cast hit?
    */
    hit: boolean;
}

/**
 * @property mass - The mass of the shape, usually in kilograms.
 * @property center - The position of the shape's centroid relative to the shape's origin.
 * @property rotationalInertia - The rotational inertia of the shape about the local origin.
 */
declare class b2MassData {
    /**
     * The mass of the shape, usually in kilograms.
    */
    mass: number;
    /**
     * The position of the shape's centroid relative to the shape's origin.
    */
    center: b2Vec2;
    /**
     * The rotational inertia of the shape about the local origin.
    */
    rotationalInertia: number;
}

/**
 * @property center - The local center
 * @property radius - The radius
 */
declare class b2Circle {
    /**
     * The local center
    */
    center: b2Vec2;
    /**
     * The radius
    */
    radius: number;
}

/**
 * @property center1 - Local center of the first semicircle
 * @property center2 - Local center of the second semicircle
 * @property radius - The radius of the semicircles
 */
declare class b2Capsule {
    /**
     * Local center of the first semicircle
    */
    center1: b2Vec2;
    /**
     * Local center of the second semicircle
    */
    center2: b2Vec2;
    /**
     * The radius of the semicircles
    */
    radius: number;
}

/**
 * @property vertices - The polygon vertices
 * @property normals - The outward normal vectors of the polygon sides
 * @property centroid - The centroid of the polygon
 * @property radius - The external radius for rounded polygons
 * @property count - The number of polygon vertices
 */
declare class b2Polygon {
    /**
     * The polygon vertices
    */
    vertices: b2Vec2[];
    /**
     * The outward normal vectors of the polygon sides
    */
    normals: b2Vec2[];
    /**
     * The centroid of the polygon
    */
    centroid: b2Vec2;
    /**
     * The external radius for rounded polygons
    */
    radius: number;
    /**
     * The number of polygon vertices
    */
    count: number;
}

/**
 * @property point1 - The first point
 * @property point2 - The second point
 */
declare class b2Segment {
    /**
     * The first point
    */
    point1: b2Vec2;
    /**
     * The second point
    */
    point2: b2Vec2;
}

/**
 * @property points - The final points of the hull
 * @property count - The number of points
 */
declare class b2Hull {
    /**
     * The final points of the hull
    */
    points: b2Vec2[];
    /**
     * The number of points
    */
    count: number;
}

/**
 * @property closest1 - The closest point on the first segment
 * @property closest2 - The closest point on the second segment
 * @property fraction1 - The barycentric coordinate on the first segment
 * @property fraction2 - The barycentric coordinate on the second segment
 * @property distanceSquared - The squared distance between the closest points
 */
declare class b2SegmentDistanceResult {
    /**
     * The closest point on the first segment
    */
    closest1: b2Vec2;
    /**
     * The closest point on the second segment
    */
    closest2: b2Vec2;
    /**
     * The barycentric coordinate on the first segment
    */
    fraction1: number;
    /**
     * The barycentric coordinate on the second segment
    */
    fraction2: number;
    /**
     * The squared distance between the closest points
    */
    distanceSquared: number;
}

/**
 * @property points - The point cloud
 * @property count - The number of points
 * @property radius - The external radius of the point cloud
 */
declare class b2DistanceProxy {
    /**
     * The point cloud
    */
    points: b2Vec2[];
    /**
     * The number of points
    */
    count: number;
    /**
     * The external radius of the point cloud
    */
    radius: number;
}

/**
 * @property count - The number of stored simplex points
 * @property indexA - The cached simplex indices on shape A
 * @property indexB - The cached simplex indices on shape B
 */
declare class b2DistanceCache {
    /**
     * The number of stored simplex points
    */
    count: number;
    /**
     * The cached simplex indices on shape A
    */
    indexA: number[];
    /**
     * The cached simplex indices on shape B
    */
    indexB: number[];
}

/**
 * @property proxyA - The proxy for shape A
 * @property proxyB - The proxy for shape B
 * @property transformA - The world transform for shape A
 * @property transformB - The world transform for shape B
 * @property useRadii - Should the proxy radius be considered?
 */
declare class b2DistanceInput {
    /**
     * The proxy for shape A
    */
    proxyA: b2DistanceProxy;
    /**
     * The proxy for shape B
    */
    proxyB: b2DistanceProxy;
    /**
     * The world transform for shape A
    */
    transformA: b2Transform;
    /**
     * The world transform for shape B
    */
    transformB: b2Transform;
    /**
     * Should the proxy radius be considered?
    */
    useRadii: boolean;
}

/**
 * @property pointA - Closest point on shapeA
 * @property pointB - Closest point on shapeB
 * @property distance - The final distance, zero if overlapped
 * @property iterations - Number of GJK iterations used
 * @property simplexCount - The number of simplexes stored in the simplex array
 */
declare class b2DistanceOutput {
    /**
     * Closest point on shapeA
    */
    pointA: b2Vec2;
    /**
     * Closest point on shapeB
    */
    pointB: b2Vec2;
    /**
     * The final distance, zero if overlapped
    */
    distance: number;
    /**
     * Number of GJK iterations used
    */
    iterations: number;
    /**
     * The number of simplexes stored in the simplex array
    */
    simplexCount: number;
}

/**
 * @property wA - Support point in proxyA
 * @property wB - Support point in proxyB
 * @property w - wB - wA
 * @property a - Barycentric coordinate for closest point
 * @property indexA - wA index
 * @property indexB - wB index
 */
declare class b2SimplexVertex {
    /**
     * Support point in proxyA
    */
    wA: b2Vec2;
    /**
     * Support point in proxyB
    */
    wB: b2Vec2;
    /**
     * wB - wA
    */
    w: b2Vec2;
    /**
     * Barycentric coordinate for closest point
    */
    a: number;
    /**
     * wA index
    */
    indexA: number;
    /**
     * wB index
    */
    indexB: number;
}

/**
 * @property v1 - Simplex vertex
 * @property v2 - Simplex vertex
 * @property v3 - Simplex vertex
 * @property count - Number of valid vertices
 */
declare class b2Simplex {
    /**
     * Simplex vertex
    */
    v1: b2SimplexVertex;
    /**
     * Simplex vertex
    */
    v2: b2SimplexVertex;
    /**
     * Simplex vertex
    */
    v3: b2SimplexVertex;
    /**
     * Number of valid vertices
    */
    count: number;
}

/**
 * @property proxyA - The proxy for shape A
 * @property proxyB - The proxy for shape B
 * @property transformA - The world transform for shape A
 * @property transformB - The world transform for shape B
 * @property translationB - The translation of shape B
 * @property maxFraction - The fraction of the translation to consider, typically 1
 */
declare class b2ShapeCastPairInput {
    /**
     * The proxy for shape A
    */
    proxyA: b2DistanceProxy;
    /**
     * The proxy for shape B
    */
    proxyB: b2DistanceProxy;
    /**
     * The world transform for shape A
    */
    transformA: b2Transform;
    /**
     * The world transform for shape B
    */
    transformB: b2Transform;
    /**
     * The translation of shape B
    */
    translationB: b2Vec2;
    /**
     * The fraction of the translation to consider, typically 1
    */
    maxFraction: number;
}

/**
 * @property localCenter - Local center of mass position
 * @property c1 - Starting center of mass world position
 * @property c2 - Ending center of mass world position
 * @property q1 - Starting world rotation
 * @property q2 - Ending world rotation
 */
declare class b2Sweep {
    /**
     * Local center of mass position
    */
    localCenter: b2Vec2;
    /**
     * Starting center of mass world position
    */
    c1: b2Vec2;
    /**
     * Ending center of mass world position
    */
    c2: b2Vec2;
    /**
     * Starting world rotation
    */
    q1: b2Rot;
    /**
     * Ending world rotation
    */
    q2: b2Rot;
}

/**
 * @property proxyA - The proxy for shape A
 * @property proxyB - The proxy for shape B
 * @property sweepA - The movement of shape A
 * @property sweepB - The movement of shape B
 * @property tMax - Defines the sweep interval [0, tMax]
 */
declare class b2TOIInput {
    /**
     * The proxy for shape A
    */
    proxyA: b2DistanceProxy;
    /**
     * The proxy for shape B
    */
    proxyB: b2DistanceProxy;
    /**
     * The movement of shape A
    */
    sweepA: b2Sweep;
    /**
     * The movement of shape B
    */
    sweepB: b2Sweep;
    /**
     * Defines the sweep interval [0, tMax]
    */
    tMax: number;
}

/**
 * @property state - The type of result
 * @property t - The time of the collision
 */
declare class b2TOIOutput {
    /**
     * The type of result
    */
    state: b2TOIState;
    /**
     * The time of the collision
    */
    t: number;
}

/**
 * @property point - Location of the contact point in world space. Subject to precision loss at large coordinates. Should only be used for debugging.
 * @property anchorA - Location of the contact point relative to bodyA's origin in world space. When used internally to the Box2D solver, this is relative to the center of mass.
 * @property anchorB - Location of the contact point relative to bodyB's origin in world space. When used internally to the Box2D solver, this is relative to the center of mass.
 * @property separation - The separation of the contact point, negative if penetrating
 * @property normalImpulse - The impulse along the manifold normal vector
 * @property tangentImpulse - The friction impulse
 * @property maxNormalImpulse - The maximum normal impulse applied during sub-stepping
 * @property normalVelocity - Relative normal velocity pre-solve. Used for hit events. If the normal impulse is zero then there was no hit. Negative means shapes are approaching.
 * @property id - Uniquely identifies a contact point between two shapes
 * @property persisted - Did this contact point exist the previous step?
 */
declare class b2ManifoldPoint {
    /**
     * Location of the contact point in world space. Subject to precision loss at large coordinates. Should only be used for debugging.
    */
    point: b2Vec2;
    /**
     * Location of the contact point relative to bodyA's origin in world space. When used internally to the Box2D solver, this is relative to the center of mass.
    */
    anchorA: b2Vec2;
    /**
     * Location of the contact point relative to bodyB's origin in world space. When used internally to the Box2D solver, this is relative to the center of mass.
    */
    anchorB: b2Vec2;
    /**
     * The separation of the contact point, negative if penetrating
    */
    separation: number;
    /**
     * The impulse along the manifold normal vector
    */
    normalImpulse: number;
    /**
     * The friction impulse
    */
    tangentImpulse: number;
    /**
     * The maximum normal impulse applied during sub-stepping
    */
    maxNormalImpulse: number;
    /**
     * Relative normal velocity pre-solve. Used for hit events. If the normal impulse is zero then there was no hit. Negative means shapes are approaching.
    */
    normalVelocity: number;
    /**
     * Uniquely identifies a contact point between two shapes
    */
    id: number;
    /**
     * Did this contact point exist the previous step?
    */
    persisted: boolean;
}

/**
 * @property points - The manifold points, up to two are possible in 2D
 * @property normal - The unit normal vector in world space, points from shape A to bodyB
 * @property pointCount - The number of contacts points, will be 0, 1, or 2
 */
declare class b2Manifold {
    /**
     * The manifold points, up to two are possible in 2D
    */
    points: b2ManifoldPoint[];
    /**
     * The unit normal vector in world space, points from shape A to bodyB
    */
    normal: b2Vec2;
    /**
     * The number of contacts points, will be 0, 1, or 2
    */
    pointCount: number;
}

/**
 * @property aabb - The node bounding box
 * @property categoryBits - Category bits for collision filtering
 * @property parent - The node parent index (allocated node)
 * @property next - The node freelist next index (free node)
 * @property child1 - Child 1 index (internal node)
 * @property child2 - Child 2 index (internal node)
 * @property userData - User data (leaf node)
 * @property height - Node height
 * @property flags - Node flags
 */
declare class b2TreeNode {
    /**
     * The node bounding box
    */
    aabb: b2AABB;
    /**
     * Category bits for collision filtering
    */
    categoryBits: number;
    /**
     * The node parent index (allocated node)
    */
    parent: number;
    /**
     * The node freelist next index (free node)
    */
    next: number;
    /**
     * Child 1 index (internal node)
    */
    child1: number;
    /**
     * Child 2 index (internal node)
    */
    child2: number;
    /**
     * User data (leaf node)
    */
    userData: number;
    /**
     * Node height
    */
    height: number;
    /**
     * Node flags
    */
    flags: number;
}

/**
 * This function is intended to set custom memory allocation and deallocation functions
 * for Box2D. However, in the Phaser Box2D JS implementation, this functionality
 * is not supported and will only generate a warning message.
 * @param allocFcn - Memory allocation function pointer
 * @param freeFcn - Memory deallocation function pointer
 */
declare function b2SetAllocator(allocFcn: (...params: any[]) => any, freeFcn: (...params: any[]) => any): void;

/**
 * This function is a stub that warns users that byte count tracking is not
 * supported in the JavaScript implementation of Box2D for Phaser.
 * @returns An integer representing the total bytes used by Box2D.
 */
declare function b2GetByteCount(): number;

/**
 * This function creates a timer object but is not supported in the Phaser Box2D JS implementation.
 * When called, it issues a console warning about lack of support.
 * @returns A timer object for measuring elapsed time.
 */
declare function b2CreateTimer(): b2Timer;

/**
 * This is a stub function that exists for compatibility with Box2D but is not
 * implemented in the Phaser Box2D JS port. It logs a warning when called.
 * @returns Returns 0 since this function not supported
 */
declare function b2GetTicks(): number;

/**
 * This function is a stub that warns that millisecond timing is not supported
 * in the Phaser Box2D JS implementation.
 * @returns The elapsed time in milliseconds.
 */
declare function b2GetMilliseconds(): number;

/**
 * This function returns the elapsed milliseconds from a Box2D timer object and resets it.
 * In the JavaScript implementation for Phaser Box2D, this functionality is not supported
 * and will trigger a warning.
 * @param timer - The Box2D timer object to query and reset
 * @returns The elapsed time in milliseconds
 */
declare function b2GetMillisecondsAndReset(timer: b2Timer): number;

/**
 * This function is a stub that issues a warning message when called, as the sleep
 * functionality is not supported in the Phaser Box2D JS implementation.
 * @param ms - The number of milliseconds to sleep
 */
declare function b2SleepMilliseconds(ms: number): void;

/**
 * This function serves as a placeholder for Box2D's b2Yield functionality, which is not supported
 * in the Phaser Box2D JS implementation. When called, it emits a warning to the console.
 */
declare function b2Yield(): void;

/**
 * This file includes code that is:
 *
 * - Copyright 2023 Erin Catto, released under the MIT license.
 * - Copyright 2024 Phaser Studio Inc, released under the MIT license.
 */
declare function b2IsPowerOf2(): void;

/**
 * Get proxy user data
 * @param tree - The dynamic tree object
 * @param proxyId - The proxy ID
 * @returns The proxy user data or 0 if the id is invalid
 */
declare function b2DynamicTree_GetUserData(tree: any, proxyId: number): number;

/**
 * @property nodes - The tree nodes
 * @property root - The root index
 * @property nodeCount - The number of nodes
 * @property nodeCapacity - The allocated node space
 * @property freeList - Node free list
 * @property proxyCount - Number of proxies created
 * @property leafIndices - Leaf indices for rebuild
 * @property leafBoxes - Leaf bounding boxes for rebuild
 * @property leafCenters - Leaf bounding box centers for rebuild
 * @property binIndices - Bins for sorting during rebuild
 * @property rebuildCapacity - Allocated space for rebuilding
 */
declare class b2DynamicTree {
    /**
     * The tree nodes
    */
    nodes: b2TreeNode[];
    /**
     * The root index
    */
    root: number;
    /**
     * The number of nodes
    */
    nodeCount: number;
    /**
     * The allocated node space
    */
    nodeCapacity: number;
    /**
     * Node free list
    */
    freeList: number;
    /**
     * Number of proxies created
    */
    proxyCount: number;
    /**
     * Leaf indices for rebuild
    */
    leafIndices: number[];
    /**
     * Leaf bounding boxes for rebuild
    */
    leafBoxes: b2AABB[];
    /**
     * Leaf bounding box centers for rebuild
    */
    leafCenters: b2Vec2[];
    /**
     * Bins for sorting during rebuild
    */
    binIndices: number[];
    /**
     * Allocated space for rebuilding
    */
    rebuildCapacity: number;
}

/**
 * @property index1 - Index value stored as unsigned 16-bit integer
 * @property revision - Revision value stored as unsigned 16-bit integer
 */
declare class b2WorldId {
    /**
     * Index value stored as unsigned 16-bit integer
    */
    index1: number;
    /**
     * Revision value stored as unsigned 16-bit integer
    */
    revision: number;
}

/**
 * @property index1 - Integer index value
 * @property world0 - 16-bit world identifier
 * @property revision - 16-bit revision number
 */
declare class b2BodyId {
    /**
     * Integer index value
    */
    index1: number;
    /**
     * 16-bit world identifier
    */
    world0: number;
    /**
     * 16-bit revision number
    */
    revision: number;
}

/**
 * @property index1 - Integer index value
 * @property world0 - 16-bit unsigned integer value
 * @property revision - 16-bit unsigned integer value
 */
declare class b2ShapeId {
    /**
     * Integer index value
    */
    index1: number;
    /**
     * 16-bit unsigned integer value
    */
    world0: number;
    /**
     * 16-bit unsigned integer value
    */
    revision: number;
}

/**
 * @property index1 - Integer index value
 * @property world0 - 16-bit world identifier
 * @property revision - 16-bit revision number
 */
declare class b2JointId {
    /**
     * Integer index value
    */
    index1: number;
    /**
     * 16-bit world identifier
    */
    world0: number;
    /**
     * 16-bit revision number
    */
    revision: number;
}

/**
 * @property index1 - Integer index value
 * @property world0 - 16-bit unsigned integer value
 * @property revision - 16-bit unsigned integer value
 */
declare class b2ChainId {
    /**
     * Integer index value
    */
    index1: number;
    /**
     * 16-bit unsigned integer value
    */
    world0: number;
    /**
     * 16-bit unsigned integer value
    */
    revision: number;
}

/**
 * Tests if a contact ID represents a null/invalid contact by checking if its index1
 * property equals 0. In Box2D, an index1 value of 0 indicates a null contact ID.
 * @param id - A contact ID object containing index1 property
 * @returns Returns true if the contact ID is null (index1 === 0), false otherwise
 */
declare function B2_IS_NULL(id: any): boolean;

/**
 * Tests whether a contact ID represents a valid contact by checking if its index1
 * property is not equal to 0. A zero value for index1 indicates a null contact ID.
 * @param id - A contact ID object containing an index1 property.
 * @returns Returns true if the contact ID is non-null (index1 !== 0), false otherwise.
 */
declare function B2_IS_NON_NULL(id: any): boolean;

/**
 * Performs a strict equality comparison between corresponding properties of two ID objects.
 * Checks equality of index1, world0, and revision properties.
 * @param id1 - First ID object containing index1, world0, and revision properties.
 * @param id2 - Second ID object containing index1, world0, and revision properties.
 * @returns True if all corresponding properties between id1 and id2 are equal, false otherwise.
 */
declare function B2_ID_EQUALS(id1: any, id2: any): boolean;

/**
 * @property x - x coordinate
 * @property y - y coordinate
 */
declare class b2Vec2 {
    /**
     * x coordinate
    */
    x: number;
    /**
     * y coordinate
    */
    y: number;
}

/**
 * @property s - The sine component of the rotation
 * @property c - The cosine component of the rotation
 */
declare class b2Rot {
    /**
     * The sine component of the rotation
    */
    s: number;
    /**
     * The cosine component of the rotation
    */
    c: number;
}

/**
 * @property p - Position vector
 * @property q - Rotation component
 */
declare class b2Transform {
    /**
     * Position vector
    */
    p: b2Vec2;
    /**
     * Rotation component
    */
    q: b2Rot;
}

/**
 * @property cy - Matrix columns
 */
declare class b2Mat22 {
    /**
     * Matrix columns
    */
    cy: b2Vec2;
}

/**
 * @property lowerBound - The lower vertex of the bounding box
 * @property upperBound - The upper vertex of the bounding box
 */
declare class b2AABB {
    /**
     * The lower vertex of the bounding box
    */
    lowerBound: b2Vec2;
    /**
     * The upper vertex of the bounding box
    */
    upperBound: b2Vec2;
}

/**
 * @param a - First number to compare
 * @param b - Second number to compare
 * @returns The smaller of the two input numbers
 */
declare function b2MinFloat(a: number, b: number): number;

/**
 * @param a - First number to compare
 * @param b - Second number to compare
 * @returns The maximum value between a and b
 */
declare function b2MaxFloat(a: number, b: number): number;

/**
 * An implementation of absolute value that returns the positive magnitude
 * of the input number.
 * @param a - The input number
 * @returns The absolute value of the input
 */
declare function b2AbsFloat(a: number): number;

/**
 * Returns lower if a < lower, upper if a > upper, otherwise returns a.
 * @param a - The value to clamp
 * @param lower - The lower bound
 * @param upper - The upper bound
 * @returns The clamped value between lower and upper bounds
 */
declare function b2ClampFloat(a: number, lower: number, upper: number): number;

/**
 * A comparison function that returns the minimum value between two integers
 * using a ternary operator.
 * @param a - First integer to compare
 * @param b - Second integer to compare
 * @returns The smaller of the two input integers
 */
declare function b2MinInt(a: number, b: number): number;

/**
 * @param a - First integer to compare
 * @param b - Second integer to compare
 * @returns The larger of the two input integers
 */
declare function b2MaxInt(a: number, b: number): number;

/**
 * Computes the absolute value of an integer using conditional logic rather than Math.abs().
 * @param a - The integer input value.
 * @returns The absolute value of the input.
 */
declare function b2AbsInt(a: number): number;

/**
 * Returns lower if a < lower, upper if a > upper, otherwise returns a.
 * @param a - The integer value to clamp
 * @param lower - The lower bound (inclusive)
 * @param upper - The upper bound (inclusive)
 * @returns The clamped integer value
 */
declare function b2ClampInt(a: number, lower: number, upper: number): number;

/**
 * Computes the dot product (scalar product) of two 2D vectors using the formula:
 * dot = a.x * b.x + a.y * b.y
 * @param a - First 2D vector.
 * @param b - Second 2D vector.
 * @returns The dot product of vectors a and b.
 */
declare function b2Dot(a: b2Vec2, b: b2Vec2): number;

/**
 * Calculates the cross product between two 2D vectors, which represents
 * the signed area of the parallelogram formed by these vectors.
 * @param a - The first 2D vector
 * @param b - The second 2D vector
 * @returns The cross product (a.x * b.y - a.y * b.x)
 */
declare function b2Cross(a: b2Vec2, b: b2Vec2): number;

/**
 * Computes the cross product of a vector and scalar, returning a new vector
 * that is perpendicular to the input vector and scaled by the scalar value.
 * @param v - The input vector
 * @param s - The scalar value
 * @returns A new vector where x = s * v.y and y = -s * v.x
 */
declare function b2CrossVS(v: b2Vec2, s: number): b2Vec2;

/**
 * Computes s × v, where × denotes the cross product.
 * The result is a new vector (-s * v.y, s * v.x).
 * @param s - The scalar value
 * @param v - The 2D vector
 * @returns A new vector perpendicular to the input vector, scaled by s
 */
declare function b2CrossSV(s: number, v: b2Vec2): b2Vec2;

/**
 * Creates a new vector that is perpendicular to the input vector by rotating it
 * 90 degrees counter-clockwise. The new vector is computed by setting the x component
 * to the negative y component of the input, and the y component to the x component
 * of the input.
 * @param v - The input vector to rotate.
 * @returns A new vector perpendicular to the input, rotated 90 degrees counter-clockwise.
 */
declare function b2LeftPerp(v: b2Vec2): b2Vec2;

/**
 * Creates a new vector that is perpendicular (rotated 90 degrees clockwise) to the input vector.
 * For a vector (x,y), returns (y,-x).
 * @param v - The input vector to rotate.
 * @returns A new vector perpendicular to the input, rotated 90 degrees clockwise.
 */
declare function b2RightPerp(v: b2Vec2): b2Vec2;

/**
 * Creates a new b2Vec2 where the x and y components are the sums of the
 * corresponding components of the input vectors.
 * @param a - The first vector.
 * @param b - The second vector.
 * @returns A new vector representing the sum of the input vectors (a + b).
 */
declare function b2Add(a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Creates a new 2D vector by subtracting the components of vector b from vector a.
 * The resulting vector has coordinates (a.x - b.x, a.y - b.y).
 * @param a - The first vector.
 * @param b - The second vector.
 * @returns A new vector representing (a - b).
 */
declare function b2Sub(a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Creates a new b2Vec2 with the negated x and y components of the input vector.
 * @param a - The input vector to negate.
 * @returns A new vector with components (-a.x, -a.y).
 */
declare function b2Neg(a: b2Vec2): b2Vec2;

/**
 * Calculates a point that lies on the straight line between vectors a and b,
 * where t=0 returns a, t=1 returns b, and values in between return proportionally
 * interpolated points.
 * @param a - The starting vector
 * @param b - The ending vector
 * @param t - The interpolation parameter between 0 and 1
 * @returns A new vector representing the interpolated point
 */
declare function b2Lerp(a: b2Vec2, b: b2Vec2, t: number): b2Vec2;

/**
 * Multiplies the x components of both vectors together and the y components of both vectors together,
 * returning a new b2Vec2 with these products as its components.
 * @param a - First 2D vector with x and y components.
 * @param b - Second 2D vector with x and y components.
 * @returns A new vector where each component is the product of the corresponding components of a and b.
 */
declare function b2Mul(a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Performs scalar multiplication on a 2D vector, where each component
 * of the vector is multiplied by the scalar value.
 * @param s - The scalar value to multiply with the vector.
 * @param v - The 2D vector to be multiplied.
 * @returns A new b2Vec2 representing the scaled vector (s * v).
 */
declare function b2MulSV(s: number, v: b2Vec2): b2Vec2;

/**
 * @param a - First vector operand
 * @param s - Scalar multiplier
 * @param b - Second vector operand
 * @returns A new vector representing the result of a + s * b
 */
declare function b2MulAdd(a: b2Vec2, s: number, b: b2Vec2): b2Vec2;

/**
 * @param a - The first vector operand
 * @param s - The scalar multiplier
 * @param b - The second vector operand
 * @returns A new vector representing the result of a - s * b
 */
declare function b2MulSub(a: b2Vec2, s: number, b: b2Vec2): b2Vec2;

/**
 * Returns a new b2Vec2 with the absolute values of the input vector's components.
 * @param a - The input vector whose components will be converted to absolute values.
 * @returns A new vector containing the absolute values of the input vector's x and y components.
 */
declare function b2Abs(a: b2Vec2): b2Vec2;

/**
 * @param a - First 2D vector
 * @param b - Second 2D vector
 * @returns A new vector with x = min(a.x, b.x) and y = min(a.y, b.y)
 */
declare function b2Min(a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Creates a new b2Vec2 where:
 * - x component is the maximum of a.x and b.x
 * - y component is the maximum of a.y and b.y
 * @param a - First input vector
 * @param b - Second input vector
 * @returns A new vector where each component is the maximum of the corresponding components from vectors a and b
 */
declare function b2Max(a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Creates a new 2D vector where each component (x,y) is clamped between
 * the corresponding components of vectors a and b. Uses b2ClampFloat
 * internally to clamp individual components.
 * @param v - The vector to clamp
 * @param a - The minimum bounds vector
 * @param b - The maximum bounds vector
 * @returns A new vector with components clamped between a and b
 */
declare function b2Clamp(v: b2Vec2, a: b2Vec2, b: b2Vec2): b2Vec2;

/**
 * Computes the Euclidean length of a 2D vector using the formula sqrt(x² + y²).
 * @param v - A 2D vector with x and y components.
 * @returns The scalar length of the vector.
 */
declare function b2Length(v: b2Vec2): number;

/**
 * Computes the dot product of a vector with itself, which gives the squared
 * length of the vector without performing a square root operation.
 * @param v - A 2D vector with x and y components.
 * @returns The squared length of the vector (x² + y²).
 */
declare function b2LengthSquared(v: b2Vec2): number;

/**
 * Computes the straight-line distance between two points using the Pythagorean theorem.
 * @param a - The first 2D vector point
 * @param b - The second 2D vector point
 * @returns The distance between points a and b
 */
declare function b2Distance(a: b2Vec2, b: b2Vec2): number;

/**
 * Computes the squared Euclidean distance between two points without taking the square root.
 * The calculation is (b.x - a.x)² + (b.y - a.y)²
 * @param a - The first 2D vector point
 * @param b - The second 2D vector point
 * @returns The squared distance between points a and b
 */
declare function b2DistanceSquared(a: b2Vec2, b: b2Vec2): number;

/**
 * Constructs a b2Rot object by computing the cosine and sine of the provided angle.
 * The resulting b2Rot contains these values as its 'c' and 's' components respectively.
 * @param angle - The rotation angle in radians
 * @returns A new b2Rot object containing the cosine and sine of the input angle
 */
declare function b2MakeRot(angle: number): b2Rot;

/**
 * Computes the magnitude of the rotation vector and divides both components by it
 * to create a normalized rotation. If the magnitude is 0, returns a default rotation.
 * @param q - A rotation object containing cosine (c) and sine (s) components
 * @returns A new normalized b2Rot object where the components form a unit vector
 */
declare function b2NormalizeRot(q: b2Rot): b2Rot;

/**
 * Verifies that the sum of squares of the sine and cosine components
 * equals 1 within a tolerance of ±6e-4, which indicates a valid rotation.
 * @param q - A rotation object containing sine (s) and cosine (c) components
 * @returns True if the rotation is normalized within a tolerance of 6e-4
 */
declare function b2IsNormalized(q: b2Rot): boolean;

/**
 * Linearly interpolates between two rotations and normalizes the result.
 * When t=0 returns q12, when t=1 returns q22.
 * @param q1 - The first rotation
 * @param q2 - The second rotation
 * @param t - Interpolation factor between 0 and 1
 * @returns The normalized interpolated rotation
 */
declare function b2NLerp(q1: b2Rot, q2: b2Rot, t: number): b2Rot;

/**
 * Performs rotation integration while ensuring the resulting rotation remains
 * normalized through magnitude scaling. The function handles the case where
 * magnitude could be zero by returning a default rotation.
 * @param q1 - The initial rotation.
 * @param deltaAngle - The angle to rotate by, in radians.
 * @returns A new normalized rotation after applying the integration.
 */
declare function b2IntegrateRotation(q1: b2Rot, deltaAngle: number): b2Rot;

/**
 * Calculates the angular velocity by comparing two rotations and dividing by the time step.
 * Uses the formula (θ2 - θ1)/dt where θ is extracted from the rotations using their sine
 * and cosine components.
 * @param q1 - The first rotation
 * @param q2 - The second rotation
 * @param inv_h - The inverse of the time step (1/dt)
 * @returns The computed angular velocity in radians per second
 */
declare function b2ComputeAngularVelocity(q1: b2Rot, q2: b2Rot, inv_h: number): number;

/**
 * Calculates the angle of a rotation by computing the arctangent of the sine
 * component divided by the cosine component using Math.atan2.
 * @param q - A rotation object containing cosine (c) and sine (s) components.
 * @returns The angle in radians, calculated using arctangent of s/c.
 */
declare function b2Rot_GetAngle(q: b2Rot): number;

/**
 * Extracts the x-axis direction vector from a rotation matrix by returning
 * a vector containing the cosine component in x and sine component in y.
 * @param q - A rotation matrix containing cosine (c) and sine (s) components
 * @returns A 2D vector representing the x-axis direction (c, s)
 */
declare function b2Rot_GetXAxis(q: b2Rot): b2Vec2;

/**
 * Extracts the Y axis vector from a rotation matrix by returning the vector (-sin, cos).
 * This represents the vertical basis vector of the rotated coordinate system.
 * @param q - A rotation matrix containing cosine (c) and sine (s) components
 * @returns A 2D vector (-sin, cos) representing the Y axis of the rotation
 */
declare function b2Rot_GetYAxis(q: b2Rot): b2Vec2;

/**
 * Performs rotation multiplication by combining the cosine and sine components
 * of two b2Rot objects using the formula:
 * result.c = q4.c * r.c - q4.s * r.s
 * result.s = q4.s * r.c + q4.c * r.s
 * @param q - First rotation
 * @param r - Second rotation
 * @returns A new rotation representing the product of the two input rotations
 */
declare function b2MulRot(q: b2Rot, r: b2Rot): b2Rot;

/**
 * Computes the product of the inverse of rotation q with rotation r.
 * For rotations, the inverse multiplication is equivalent to using the transpose
 * of the rotation matrix.
 * @param q - The first rotation to be inverted
 * @param r - The second rotation to be multiplied
 * @returns A new rotation representing q^-1 * r
 */
declare function b2InvMulRot(q: b2Rot, r: b2Rot): b2Rot;

/**
 * Computes the angle between two rotations by using their sine and cosine components.
 * The result is the angle needed to rotate from rotation 'a' to rotation 'b'.
 * @param b - The first rotation
 * @param a - The second rotation
 * @returns The relative angle in radians between the two rotations
 */
declare function b2RelativeAngle(b: b2Rot, a: b2Rot): number;

/**
 * This function takes an angle in radians and ensures it falls within the range [-π, π]
 * by adding or subtracting 2π as needed. If the input angle is already within
 * the valid range, it is returned unchanged.
 * @param angle - The input angle in radians to normalize.
 * @returns The normalized angle in radians within the range [-π, π].
 */
declare function b2UnwindAngle(angle: number): number;

/**
 * Applies a 2D rotation to a vector using the formula:
 * x' = c*x - s*y
 * y' = s*x + c*y
 * where (x,y) is the input vector and (c,s) represents the rotation
 * @param q - A rotation object containing cosine (c) and sine (s) components
 * @param v - The vector to rotate
 * @returns A new vector representing the rotated result
 */
declare function b2RotateVector(q: b2Rot, v: b2Vec2): b2Vec2;

/**
 * Applies the inverse of the rotation defined by q4 to vector v.
 * The operation is equivalent to multiplying the vector by the transpose
 * of the rotation matrix represented by q4.
 * @param q - A rotation matrix containing cosine (c) and sine (s) components
 * @param v - The vector to be inversely rotated
 * @returns A new vector representing the inverse rotation of v by q4
 */
declare function b2InvRotateVector(q: b2Rot, v: b2Vec2): b2Vec2;

/**
 * Applies a rigid body transformation to a 2D point. The transformation consists of
 * a rotation followed by a translation. The rotation is applied using the rotation
 * matrix stored in t.q (cosine and sine components), and the translation is applied
 * using the position vector stored in t.p.
 * @param t - A transform containing position (p) and rotation (q) components
 * @param p - The point to transform
 * @returns The transformed point
 */
declare function b2TransformPoint(t: b2Transform, p: b2Vec2): b2Vec2;

/**
 * Computes the inverse transform of a point by first translating relative to the transform's
 * position, then applying the inverse rotation. The rotation inverse is computed using
 * the transpose of the rotation matrix.
 * @param t - A transform containing position (p) and rotation (q) components
 * @param p - The point to transform
 * @returns The inverse transformed point
 */
declare function b2InvTransformPoint(t: b2Transform, p: b2Vec2): b2Vec2;

/**
 * Combines two transforms by first multiplying their rotations (q components),
 * then rotating B's position vector by A's rotation and adding it to A's position.
 * The result represents the combined transformation of first applying B, then A.
 * @param A - The first transform
 * @param B - The second transform
 * @returns A new transform C that represents the multiplication of A and B
 */
declare function b2MulTransforms(A: b2Transform, B: b2Transform): b2Transform;

/**
 * Calculates A^-1 * B, where A^-1 is the inverse of transform A.
 * The result combines both the rotational and translational components
 * of the transforms using their quaternion and position vectors.
 * @param A - The first transform
 * @param B - The second transform
 * @returns A new transform representing the inverse multiplication of A and B
 */
declare function b2InvMulTransforms(A: b2Transform, B: b2Transform): b2Transform;

/**
 * Performs matrix-vector multiplication of the form Av, where A is a 2x2 matrix
 * and v is a 2D vector. The result is a new 2D vector.
 * @param A - A 2x2 matrix with components cx and cy, each containing x and y values
 * @param v - A 2D vector with x and y components
 * @returns The resulting 2D vector from the matrix-vector multiplication
 */
declare function b2MulMV(A: b2Mat22, v: b2Vec2): b2Vec2;

/**
 * Computes the inverse of a 2x2 matrix using the formula:
 * For matrix [[a,b],[c,d]], inverse = (1/det) * [[d,-c],[-b,a]]
 * where det = ad-bc
 * @param A - The input 2x2 matrix to invert
 * @returns The inverse of matrix A. If the determinant is 0, returns a matrix with undefined values
 */
declare function b2GetInverse22(A: b2Mat22): b2Mat22;

/**
 * Solves the system using the formula:
 * x = (1/det) * [a22 -a12] * [b.x]
 * [-a21  a11]   [b.y]
 * where det = a11*a22 - a12*a21
 * @param A - A 2x2 matrix represented as two column vectors (cx, cy)
 * @param b - A 2D vector representing the right-hand side of the equation
 * @returns The solution vector x that satisfies Ax = b. Returns a zero vector if the system is singular (det = 0)
 */
declare function b2Solve22(A: b2Mat22, b: b2Vec2): b2Vec2;

/**
 * Tests if AABB 'b' is completely contained within AABB 'a' by comparing their bounds.
 * An AABB contains another if its lower bounds are less than or equal to the other's lower bounds
 * and its upper bounds are greater than or equal to the other's upper bounds.
 * @param a - The containing AABB
 * @param b - The AABB to test for containment
 * @returns True if AABB 'a' completely contains AABB 'b', false otherwise
 */
declare function b2AABB_Contains(a: b2AABB, b: b2AABB): boolean;

/**
 * Computes the center point of an AABB by averaging its lower and upper bounds
 * in both X and Y dimensions.
 * @param a - The AABB object containing lowerBound and upperBound coordinates.
 * @returns A 2D vector representing the center point of the AABB.
 */
declare function b2AABB_Center(a: b2AABB): b2Vec2;

/**
 * Computes the extents of an AABB by calculating half the difference between
 * its upper and lower bounds in both x and y dimensions.
 * @param a - The AABB to calculate extents for
 * @returns A vector containing the half-width and half-height of the AABB
 */
declare function b2AABB_Extents(a: b2AABB): b2Vec2;

/**
 * Computes the union of two axis-aligned bounding boxes by creating a new AABB
 * with a lower bound at the minimum coordinates and an upper bound at the
 * maximum coordinates of both input boxes.
 * @param a - The first axis-aligned bounding box
 * @param b - The second axis-aligned bounding box
 * @returns A new AABB that encompasses both input boxes
 */
declare function b2AABB_Union(a: b2AABB, b: b2AABB): b2AABB;

/**
 * This function performs a validation check on a number by ensuring it is both
 * finite and not NaN (Not a Number).
 * @param a - The number to validate.
 * @returns True if the number is valid (finite and not NaN), false otherwise.
 */
declare function b2IsValid(a: number): boolean;

/**
 * Validates a b2Vec2 object by checking if it exists and its components are valid numbers.
 * @param v - The vector to validate, containing x and y components.
 * @returns True if the vector exists and both x and y components are valid numbers.
 */
declare function b2Vec2_IsValid(v: b2Vec2): boolean;

/**
 * Checks if a b2Rot object is valid by verifying:
 * 1. The object exists
 * 2. Both sine and cosine components contain valid numbers
 * 3. The rotation is properly normalized (s² + c² = 1)
 * @param q - A rotation object containing sine (s) and cosine (c) components
 * @returns True if the rotation is valid, false otherwise
 */
declare function b2Rot_IsValid(q: b2Rot): boolean;

/**
 * Checks if an AABB is valid by verifying:
 * 1. The AABB object exists
 * 2. The width (upperBoundX - lowerBoundX) is non-negative
 * 3. The height (upperBoundY - lowerBoundY) is non-negative
 * 4. All coordinate values are valid numbers
 * @param aabb - The AABB to validate
 * @returns True if the AABB exists and has valid dimensions and coordinates
 */
declare function b2AABB_IsValid(aabb: b2AABB): boolean;

/**
 * Normalizes the input vector by dividing its components by its length.
 * If the vector's length is greater than the epsilon value, the function
 * returns a new vector with the same direction but unit length.
 * @param v - The vector to normalize.
 * @returns Returns a new normalized b2Vec2 if successful.
 * If the vector length is less than epsilon, returns a zero vector (0,0).
 */
declare function b2Normalize(v: b2Vec2): b2Vec2;

/**
 * Normalizes a 2D vector and performs length validation.
 * @param v - The vector to normalize.
 * @returns A new normalized vector with unit length.
 */
declare function b2NormalizeChecked(v: b2Vec2): b2Vec2;

/**
 * This file includes code that is:
 *
 * - Copyright 2023 Erin Catto, released under the MIT license.
 * - Copyright 2024 Phaser Studio Inc, released under the MIT license.
 */
declare const B2_SHAPE_PAIR_KEY: any;

/**
 * Result from b2World_RayCastClosest
 * @property shapeId - The shape that was hit
 * @property point - The hit point
 * @property normal - The hit normal
 * @property fraction - The hit fraction along the ray
 * @property nodeVisits - Number of tree nodes visited
 * @property leafVisits - Number of tree leaves visited
 * @property hit - Whether the ray hit anything
 */
declare class b2RayResult {
    /**
     * The shape that was hit
    */
    shapeId: b2ShapeId;
    /**
     * The hit point
    */
    point: b2Vec2;
    /**
     * The hit normal
    */
    normal: b2Vec2;
    /**
     * The hit fraction along the ray
    */
    fraction: number;
    /**
     * Number of tree nodes visited
    */
    nodeVisits: number;
    /**
     * Number of tree leaves visited
    */
    leafVisits: number;
    /**
     * Whether the ray hit anything
    */
    hit: boolean;
}

/**
 * @property gravity - Gravity vector. Box2D has no up-vector defined.
 * @property restitutionThreshold - Restitution velocity threshold, usually in m/s. Collisions above this speed have restitution applied (will bounce).
 * @property contactPushoutVelocity - This parameter controls how fast overlap is resolved and has units of meters per second
 * @property hitEventThreshold - Threshold velocity for hit events. Usually meters per second.
 * @property contactHertz - Contact stiffness. Cycles per second.
 * @property contactDampingRatio - Contact bounciness. Non-dimensional.
 * @property jointHertz - Joint stiffness. Cycles per second.
 * @property jointDampingRatio - Joint bounciness. Non-dimensional.
 * @property maximumLinearVelocity - Maximum linear velocity. Usually meters per second.
 * @property frictionMixingRule - Mixing rule for friction. Default is b2_mixGeometricMean.
 * @property restitutionMixingRule - Mixing rule for restitution. Default is b2_mixMaximum.
 * @property enableSleep - Can bodies go to sleep to improve performance
 * @property enableContinuous - Enable continuous collision
 * @property workerCount - Number of workers to use with the provided task system.
 * @property enqueueTask - Function to spawn tasks
 * @property finishTask - Function to finish a task
 * @property userTaskContext - User context that is provided to enqueueTask and finishTask
 * @property userData - User data
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2WorldDef {
    /**
     * Gravity vector. Box2D has no up-vector defined.
    */
    gravity: b2Vec2;
    /**
     * Restitution velocity threshold, usually in m/s. Collisions above this speed have restitution applied (will bounce).
    */
    restitutionThreshold: number;
    /**
     * This parameter controls how fast overlap is resolved and has units of meters per second
    */
    contactPushoutVelocity: number;
    /**
     * Threshold velocity for hit events. Usually meters per second.
    */
    hitEventThreshold: number;
    /**
     * Contact stiffness. Cycles per second.
    */
    contactHertz: number;
    /**
     * Contact bounciness. Non-dimensional.
    */
    contactDampingRatio: number;
    /**
     * Joint stiffness. Cycles per second.
    */
    jointHertz: number;
    /**
     * Joint bounciness. Non-dimensional.
    */
    jointDampingRatio: number;
    /**
     * Maximum linear velocity. Usually meters per second.
    */
    maximumLinearVelocity: number;
    /**
     * Mixing rule for friction. Default is b2_mixGeometricMean.
    */
    frictionMixingRule: b2MixingRule;
    /**
     * Mixing rule for restitution. Default is b2_mixMaximum.
    */
    restitutionMixingRule: b2MixingRule;
    /**
     * Can bodies go to sleep to improve performance
    */
    enableSleep: boolean;
    /**
     * Enable continuous collision
    */
    enableContinuous: boolean;
    /**
     * Number of workers to use with the provided task system.
    */
    workerCount: number;
    /**
     * Function to spawn tasks
    */
    enqueueTask: (...params: any[]) => any;
    /**
     * Function to finish a task
    */
    finishTask: (...params: any[]) => any;
    /**
     * User context that is provided to enqueueTask and finishTask
    */
    userTaskContext: any;
    /**
     * User data
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property type - The body type: static, kinematic, or dynamic
 * @property position - The initial world position of the body
 * @property rotation - The initial world rotation of the body
 * @property linearVelocity - The initial linear velocity in meters per second
 * @property angularVelocity - The initial angular velocity in radians per second
 * @property linearDamping - Linear damping to reduce linear velocity
 * @property angularDamping - Angular damping to reduce angular velocity
 * @property gravityScale - Scale factor applied to gravity for this body
 * @property sleepThreshold - Sleep velocity threshold, default 0.05 m/s
 * @property userData - Application specific body data
 * @property enableSleep - Whether this body can fall asleep
 * @property isAwake - Whether body starts awake or sleeping
 * @property fixedRotation - Whether to prevent rotation
 * @property isBullet - Whether to use continuous collision detection
 * @property isEnabled - Whether body can move and collide
 * @property allowFastRotation - Whether to bypass rotation speed limits
 * @property internalValue - Internal validation flag
 */
declare class b2BodyDef {
    /**
     * The body type: static, kinematic, or dynamic
    */
    type: b2BodyType;
    /**
     * The initial world position of the body
    */
    position: b2Vec2;
    /**
     * The initial world rotation of the body
    */
    rotation: b2Rot;
    /**
     * The initial linear velocity in meters per second
    */
    linearVelocity: b2Vec2;
    /**
     * The initial angular velocity in radians per second
    */
    angularVelocity: number;
    /**
     * Linear damping to reduce linear velocity
    */
    linearDamping: number;
    /**
     * Angular damping to reduce angular velocity
    */
    angularDamping: number;
    /**
     * Scale factor applied to gravity for this body
    */
    gravityScale: number;
    /**
     * Sleep velocity threshold, default 0.05 m/s
    */
    sleepThreshold: number;
    /**
     * Application specific body data
    */
    userData: any;
    /**
     * Whether this body can fall asleep
    */
    enableSleep: boolean;
    /**
     * Whether body starts awake or sleeping
    */
    isAwake: boolean;
    /**
     * Whether to prevent rotation
    */
    fixedRotation: boolean;
    /**
     * Whether to use continuous collision detection
    */
    isBullet: boolean;
    /**
     * Whether body can move and collide
    */
    isEnabled: boolean;
    /**
     * Whether to bypass rotation speed limits
    */
    allowFastRotation: boolean;
    /**
     * Internal validation flag
    */
    internalValue: number;
}

/**
 * @property userData - Use this to store application specific shape data
 * @property friction - The Coulomb (dry) friction coefficient, usually in the range [0,1]
 * @property restitution - The restitution (bounce) usually in the range [0,1]
 * @property density - The density, usually in kg/m^2
 * @property filter - Collision filtering data
 * @property customColor - Custom debug draw color
 * @property isSensor - A sensor shape generates overlap events but never generates a collision response
 * @property enableSensorEvents - Enable sensor events for this shape. Only applies to kinematic and dynamic bodies
 * @property enableContactEvents - Enable contact events for this shape. Only applies to kinematic and dynamic bodies
 * @property enableHitEvents - Enable hit events for this shape. Only applies to kinematic and dynamic bodies
 * @property enablePreSolveEvents - Enable pre-solve contact events for this shape. Only applies to dynamic bodies
 * @property invokeContactCreation - Override static body behavior to force contact creation
 * @property updateBodyMass - Should the body update mass properties when shape is created
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET
 */
declare class b2ShapeDef {
    /**
     * Use this to store application specific shape data
    */
    userData: any;
    /**
     * The Coulomb (dry) friction coefficient, usually in the range [0,1]
    */
    friction: number;
    /**
     * The restitution (bounce) usually in the range [0,1]
    */
    restitution: number;
    /**
     * The density, usually in kg/m^2
    */
    density: number;
    /**
     * Collision filtering data
    */
    filter: b2Filter;
    /**
     * Custom debug draw color
    */
    customColor: number;
    /**
     * A sensor shape generates overlap events but never generates a collision response
    */
    isSensor: boolean;
    /**
     * Enable sensor events for this shape. Only applies to kinematic and dynamic bodies
    */
    enableSensorEvents: boolean;
    /**
     * Enable contact events for this shape. Only applies to kinematic and dynamic bodies
    */
    enableContactEvents: boolean;
    /**
     * Enable hit events for this shape. Only applies to kinematic and dynamic bodies
    */
    enableHitEvents: boolean;
    /**
     * Enable pre-solve contact events for this shape. Only applies to dynamic bodies
    */
    enablePreSolveEvents: boolean;
    /**
     * Override static body behavior to force contact creation
    */
    invokeContactCreation: boolean;
    /**
     * Should the body update mass properties when shape is created
    */
    updateBodyMass: boolean;
    /**
     * Used internally to detect a valid definition. DO NOT SET
    */
    internalValue: number;
}

/**
 * @property userData - Use this to store application specific shape data
 * @property points - Array of at least 4 points defining the chain segments
 * @property count - The point count, must be 4 or more
 * @property friction - The friction coefficient, usually in the range [0,1]
 * @property restitution - The restitution (elasticity) usually in the range [0,1]
 * @property filter - Contact filtering data
 * @property customColor - Custom debug draw color
 * @property isLoop - Indicates a closed chain formed by connecting first and last points
 * @property internalValue - Used internally to detect valid definition. DO NOT SET
 */
declare class b2ChainDef {
    /**
     * Use this to store application specific shape data
    */
    userData: void;
    /**
     * Array of at least 4 points defining the chain segments
    */
    points: b2Vec2[];
    /**
     * The point count, must be 4 or more
    */
    count: number;
    /**
     * The friction coefficient, usually in the range [0,1]
    */
    friction: number;
    /**
     * The restitution (elasticity) usually in the range [0,1]
    */
    restitution: number;
    /**
     * Contact filtering data
    */
    filter: b2Filter;
    /**
     * Custom debug draw color
    */
    customColor: number;
    /**
     * Indicates a closed chain formed by connecting first and last points
    */
    isLoop: boolean;
    /**
     * Used internally to detect valid definition. DO NOT SET
    */
    internalValue: number;
}

/**
 * This requires defining an anchor point on both bodies and the non-zero distance of the distance joint. The definition uses local anchor points so that the initial configuration can violate the constraint slightly. This helps when saving and loading a game.
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property localAnchorA - The local anchor point relative to bodyA's origin
 * @property localAnchorB - The local anchor point relative to bodyB's origin
 * @property length - The rest length of this joint. Clamped to a stable minimum value.
 * @property enableSpring - Enable the distance constraint to behave like a spring. If false then the distance joint will be rigid, overriding the limit and motor.
 * @property hertz - The spring linear stiffness Hertz, cycles per second
 * @property dampingRatio - The spring linear damping ratio, non-dimensional
 * @property enableLimit - Enable/disable the joint limit
 * @property minLength - Minimum length. Clamped to a stable minimum value.
 * @property maxLength - Maximum length. Must be greater than or equal to the minimum length.
 * @property enableMotor - Enable/disable the joint motor
 * @property maxMotorForce - The maximum motor force, usually in newtons
 * @property motorSpeed - The desired motor speed, usually in meters per second
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2DistanceJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The local anchor point relative to bodyA's origin
    */
    localAnchorA: b2Vec2;
    /**
     * The local anchor point relative to bodyB's origin
    */
    localAnchorB: b2Vec2;
    /**
     * The rest length of this joint. Clamped to a stable minimum value.
    */
    length: number;
    /**
     * Enable the distance constraint to behave like a spring. If false then the distance joint will be rigid, overriding the limit and motor.
    */
    enableSpring: boolean;
    /**
     * The spring linear stiffness Hertz, cycles per second
    */
    hertz: number;
    /**
     * The spring linear damping ratio, non-dimensional
    */
    dampingRatio: number;
    /**
     * Enable/disable the joint limit
    */
    enableLimit: boolean;
    /**
     * Minimum length. Clamped to a stable minimum value.
    */
    minLength: number;
    /**
     * Maximum length. Must be greater than or equal to the minimum length.
    */
    maxLength: number;
    /**
     * Enable/disable the joint motor
    */
    enableMotor: boolean;
    /**
     * The maximum motor force, usually in newtons
    */
    maxMotorForce: number;
    /**
     * The desired motor speed, usually in meters per second
    */
    motorSpeed: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property linearOffset - Position of bodyB minus the position of bodyA, in bodyA's frame
 * @property angularOffset - The bodyB angle minus bodyA angle in radians
 * @property maxForce - The maximum motor force in newtons
 * @property maxTorque - The maximum motor torque in newton-meters
 * @property correctionFactor - Position correction factor in the range [0,1]
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2MotorJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * Position of bodyB minus the position of bodyA, in bodyA's frame
    */
    linearOffset: b2Vec2;
    /**
     * The bodyB angle minus bodyA angle in radians
    */
    angularOffset: number;
    /**
     * The maximum motor force in newtons
    */
    maxForce: number;
    /**
     * The maximum motor torque in newton-meters
    */
    maxTorque: number;
    /**
     * Position correction factor in the range [0,1]
    */
    correctionFactor: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property target - The initial target point in world space
 * @property hertz - Stiffness in hertz
 * @property dampingRatio - Damping ratio, non-dimensional
 * @property maxForce - Maximum force, typically in newtons
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2MouseJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The initial target point in world space
    */
    target: b2Vec2;
    /**
     * Stiffness in hertz
    */
    hertz: number;
    /**
     * Damping ratio, non-dimensional
    */
    dampingRatio: number;
    /**
     * Maximum force, typically in newtons
    */
    maxForce: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property localAnchorA - The local anchor point relative to bodyA's origin
 * @property localAnchorB - The local anchor point relative to bodyB's origin
 * @property localAxisA - The local translation unit axis in bodyA
 * @property referenceAngle - The constrained angle between the bodies: bodyB_angle - bodyA_angle
 * @property enableSpring - Enable a linear spring along the prismatic joint axis
 * @property hertz - The spring stiffness Hertz, cycles per second
 * @property dampingRatio - The spring damping ratio, non-dimensional
 * @property enableLimit - Enable/disable the joint limit
 * @property lowerTranslation - The lower translation limit
 * @property upperTranslation - The upper translation limit
 * @property enableMotor - Enable/disable the joint motor
 * @property maxMotorForce - The maximum motor force, typically in newtons
 * @property motorSpeed - The desired motor speed, typically in meters per second
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2PrismaticJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The local anchor point relative to bodyA's origin
    */
    localAnchorA: b2Vec2;
    /**
     * The local anchor point relative to bodyB's origin
    */
    localAnchorB: b2Vec2;
    /**
     * The local translation unit axis in bodyA
    */
    localAxisA: b2Vec2;
    /**
     * The constrained angle between the bodies: bodyB_angle - bodyA_angle
    */
    referenceAngle: number;
    /**
     * Enable a linear spring along the prismatic joint axis
    */
    enableSpring: boolean;
    /**
     * The spring stiffness Hertz, cycles per second
    */
    hertz: number;
    /**
     * The spring damping ratio, non-dimensional
    */
    dampingRatio: number;
    /**
     * Enable/disable the joint limit
    */
    enableLimit: boolean;
    /**
     * The lower translation limit
    */
    lowerTranslation: number;
    /**
     * The upper translation limit
    */
    upperTranslation: number;
    /**
     * Enable/disable the joint motor
    */
    enableMotor: boolean;
    /**
     * The maximum motor force, typically in newtons
    */
    maxMotorForce: number;
    /**
     * The desired motor speed, typically in meters per second
    */
    motorSpeed: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property localAnchorA - The local anchor point relative to bodyA's origin
 * @property localAnchorB - The local anchor point relative to bodyB's origin
 * @property referenceAngle - The bodyB angle minus bodyA angle in the reference state (radians)
 * @property enableSpring - Enable a rotational spring on the revolute hinge axis
 * @property hertz - The spring stiffness Hertz, cycles per second
 * @property dampingRatio - The spring damping ratio, non-dimensional
 * @property enableLimit - A flag to enable joint limits
 * @property lowerAngle - The lower angle for the joint limit in radians
 * @property upperAngle - The upper angle for the joint limit in radians
 * @property enableMotor - A flag to enable the joint motor
 * @property maxMotorTorque - The maximum motor torque, typically in newton-meters
 * @property motorSpeed - The desired motor speed in radians per second
 * @property drawSize - Scale the debug draw
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2RevoluteJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The local anchor point relative to bodyA's origin
    */
    localAnchorA: b2Vec2;
    /**
     * The local anchor point relative to bodyB's origin
    */
    localAnchorB: b2Vec2;
    /**
     * The bodyB angle minus bodyA angle in the reference state (radians)
    */
    referenceAngle: number;
    /**
     * Enable a rotational spring on the revolute hinge axis
    */
    enableSpring: boolean;
    /**
     * The spring stiffness Hertz, cycles per second
    */
    hertz: number;
    /**
     * The spring damping ratio, non-dimensional
    */
    dampingRatio: number;
    /**
     * A flag to enable joint limits
    */
    enableLimit: boolean;
    /**
     * The lower angle for the joint limit in radians
    */
    lowerAngle: number;
    /**
     * The upper angle for the joint limit in radians
    */
    upperAngle: number;
    /**
     * A flag to enable the joint motor
    */
    enableMotor: boolean;
    /**
     * The maximum motor torque, typically in newton-meters
    */
    maxMotorTorque: number;
    /**
     * The desired motor speed in radians per second
    */
    motorSpeed: number;
    /**
     * Scale the debug draw
    */
    drawSize: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property localAnchorA - The local anchor point relative to bodyA's origin
 * @property localAnchorB - The local anchor point relative to bodyB's origin
 * @property referenceAngle - The bodyB angle minus bodyA angle in the reference state (radians)
 * @property linearHertz - Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
 * @property angularHertz - Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
 * @property linearDampingRatio - Linear damping ratio, non-dimensional. Use 1 for critical damping.
 * @property angularDampingRatio - Linear damping ratio, non-dimensional. Use 1 for critical damping.
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2WeldJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The local anchor point relative to bodyA's origin
    */
    localAnchorA: b2Vec2;
    /**
     * The local anchor point relative to bodyB's origin
    */
    localAnchorB: b2Vec2;
    /**
     * The bodyB angle minus bodyA angle in the reference state (radians)
    */
    referenceAngle: number;
    /**
     * Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
    */
    linearHertz: number;
    /**
     * Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
    */
    angularHertz: number;
    /**
     * Linear damping ratio, non-dimensional. Use 1 for critical damping.
    */
    linearDampingRatio: number;
    /**
     * Linear damping ratio, non-dimensional. Use 1 for critical damping.
    */
    angularDampingRatio: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property bodyIdA - The first attached body
 * @property bodyIdB - The second attached body
 * @property localAnchorA - The local anchor point relative to bodyA's origin
 * @property localAnchorB - The local anchor point relative to bodyB's origin
 * @property localAxisA - The local translation unit axis in bodyA
 * @property enableSpring - Enable a linear spring along the local axis
 * @property hertz - Spring stiffness in Hertz
 * @property dampingRatio - Spring damping ratio, non-dimensional
 * @property enableLimit - Enable/disable the joint linear limit
 * @property lowerTranslation - The lower translation limit
 * @property upperTranslation - The upper translation limit
 * @property enableMotor - Enable/disable the joint rotational motor
 * @property maxMotorTorque - The maximum motor torque, typically in newton-meters
 * @property motorSpeed - The desired motor speed in radians per second
 * @property collideConnected - Set this flag to true if the attached bodies should collide
 * @property userData - User data pointer
 * @property internalValue - Used internally to detect a valid definition. DO NOT SET.
 */
declare class b2WheelJointDef {
    /**
     * The first attached body
    */
    bodyIdA: b2BodyId;
    /**
     * The second attached body
    */
    bodyIdB: b2BodyId;
    /**
     * The local anchor point relative to bodyA's origin
    */
    localAnchorA: b2Vec2;
    /**
     * The local anchor point relative to bodyB's origin
    */
    localAnchorB: b2Vec2;
    /**
     * The local translation unit axis in bodyA
    */
    localAxisA: b2Vec2;
    /**
     * Enable a linear spring along the local axis
    */
    enableSpring: boolean;
    /**
     * Spring stiffness in Hertz
    */
    hertz: number;
    /**
     * Spring damping ratio, non-dimensional
    */
    dampingRatio: number;
    /**
     * Enable/disable the joint linear limit
    */
    enableLimit: boolean;
    /**
     * The lower translation limit
    */
    lowerTranslation: number;
    /**
     * The upper translation limit
    */
    upperTranslation: number;
    /**
     * Enable/disable the joint rotational motor
    */
    enableMotor: boolean;
    /**
     * The maximum motor torque, typically in newton-meters
    */
    maxMotorTorque: number;
    /**
     * The desired motor speed in radians per second
    */
    motorSpeed: number;
    /**
     * Set this flag to true if the attached bodies should collide
    */
    collideConnected: boolean;
    /**
     * User data pointer
    */
    userData: any;
    /**
     * Used internally to detect a valid definition. DO NOT SET.
    */
    internalValue: number;
}

/**
 * @property sensorShapeId - The id of the sensor shape
 * @property visitorShapeId - The id of the dynamic shape that began touching the sensor shape
 */
declare class b2SensorBeginTouchEvent {
    /**
     * The id of the sensor shape
    */
    sensorShapeId: b2ShapeId;
    /**
     * The id of the dynamic shape that began touching the sensor shape
    */
    visitorShapeId: b2ShapeId;
}

/**
 * @property sensorShapeId - The id of the sensor shape
 * @property visitorShapeId - The id of the dynamic shape that stopped touching the sensor shape
 */
declare class b2SensorEndTouchEvent {
    /**
     * The id of the sensor shape
    */
    sensorShapeId: b2ShapeId;
    /**
     * The id of the dynamic shape that stopped touching the sensor shape
    */
    visitorShapeId: b2ShapeId;
}

/**
 * @property beginEvents - Array of sensor begin touch events
 * @property endEvents - Array of sensor end touch events
 * @property beginCount - The number of begin touch events
 * @property endCount - The number of end touch events
 */
declare class b2SensorEvents {
    /**
     * Array of sensor begin touch events
    */
    beginEvents: b2SensorBeginTouchEvent[];
    /**
     * Array of sensor end touch events
    */
    endEvents: b2SensorEndTouchEvent[];
    /**
     * The number of begin touch events
    */
    beginCount: number;
    /**
     * The number of end touch events
    */
    endCount: number;
}

/**
 * @property shapeIdA - Id of the first shape
 * @property shapeIdB - Id of the second shape
 * @property manifold - The initial contact manifold
 */
declare class b2ContactBeginTouchEvent {
    /**
     * Id of the first shape
    */
    shapeIdA: b2ShapeId;
    /**
     * Id of the second shape
    */
    shapeIdB: b2ShapeId;
    /**
     * The initial contact manifold
    */
    manifold: b2Manifold;
}

/**
 * @property shapeIdA - Id of the first shape
 * @property shapeIdB - Id of the second shape
 */
declare class b2ContactEndTouchEvent {
    /**
     * Id of the first shape
    */
    shapeIdA: b2ShapeId;
    /**
     * Id of the second shape
    */
    shapeIdB: b2ShapeId;
}

/**
 * @property shapeIdA - Id of the first shape
 * @property shapeIdB - Id of the second shape
 * @property point - Point where the shapes hit
 * @property normal - Normal vector pointing from shape A to shape B
 * @property approachSpeed - The speed the shapes are approaching. Always positive. Typically in meters per second.
 */
declare class b2ContactHitEvent {
    /**
     * Id of the first shape
    */
    shapeIdA: b2ShapeId;
    /**
     * Id of the second shape
    */
    shapeIdB: b2ShapeId;
    /**
     * Point where the shapes hit
    */
    point: b2Vec2;
    /**
     * Normal vector pointing from shape A to shape B
    */
    normal: b2Vec2;
    /**
     * The speed the shapes are approaching. Always positive. Typically in meters per second.
    */
    approachSpeed: number;
}

/**
 * @property beginEvents - Array of begin touch events
 * @property endEvents - Array of end touch events
 * @property hitEvents - Array of hit events
 * @property beginCount - Number of begin touch events
 * @property endCount - Number of end touch events
 * @property hitCount - Number of hit events
 */
declare class b2ContactEvents {
    /**
     * Array of begin touch events
    */
    beginEvents: b2ContactBeginTouchEvent[];
    /**
     * Array of end touch events
    */
    endEvents: b2ContactEndTouchEvent[];
    /**
     * Array of hit events
    */
    hitEvents: b2ContactHitEvent[];
    /**
     * Number of begin touch events
    */
    beginCount: number;
    /**
     * Number of end touch events
    */
    endCount: number;
    /**
     * Number of hit events
    */
    hitCount: number;
}

/**
 * @property transform - The new transform of the moved body
 * @property bodyId - The identifier of the moved body
 * @property userData - User data associated with the body
 * @property fellAsleep - Indicates if the body transitioned to sleep state
 */
declare class b2BodyMoveEvent {
    /**
     * The new transform of the moved body
    */
    transform: b2Transform;
    /**
     * The identifier of the moved body
    */
    bodyId: b2BodyId;
    /**
     * User data associated with the body
    */
    userData: void;
    /**
     * Indicates if the body transitioned to sleep state
    */
    fellAsleep: boolean;
}

/**
 * @property moveEvents - Array of move events
 * @property moveCount - Number of move events
 */
declare class b2BodyEvents {
    /**
     * Array of move events
    */
    moveEvents: b2BodyMoveEvent[];
    /**
     * Number of move events
    */
    moveCount: number;
}

/**
 * @property shapeIdA - The identifier for the first shape
 * @property shapeIdB - The identifier for the second shape
 * @property manifold - The contact manifold between the shapes
 */
declare class b2ContactData {
    /**
     * The identifier for the first shape
    */
    shapeIdA: b2ShapeId;
    /**
     * The identifier for the second shape
    */
    shapeIdB: b2ShapeId;
    /**
     * The contact manifold between the shapes
    */
    manifold: b2Manifold;
}

/**
 * @property DrawString - Draw a string
 * @property drawingBounds - Bounds to use if restricting drawing to a rectangular region
 * @property useDrawingBounds - Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting
 * @property drawShapes - Option to draw shapes
 * @property drawJoints - Option to draw joints
 * @property drawJointExtras - Option to draw additional information for joints
 * @property drawAABBs - Option to draw the bounding boxes for shapes
 * @property drawMass - Option to draw the mass and center of mass of dynamic bodies
 * @property drawContacts - Option to draw contact points
 * @property drawGraphColors - Option to visualize the graph coloring used for contacts and joints
 * @property drawContactNormals - Option to draw contact normals
 * @property drawContactImpulses - Option to draw contact normal impulses
 * @property drawFrictionImpulses - Option to draw contact friction impulses
 * @property context - User context that is passed as an argument to drawing callback functions
 */
declare class b2DebugDraw {
    /**
     * Draw a string
    */
    DrawString: (...params: any[]) => any;
    /**
     * Bounds to use if restricting drawing to a rectangular region
    */
    drawingBounds: b2AABB;
    /**
     * Option to restrict drawing to a rectangular region. May suffer from unstable depth sorting
    */
    useDrawingBounds: boolean;
    /**
     * Option to draw shapes
    */
    drawShapes: boolean;
    /**
     * Option to draw joints
    */
    drawJoints: boolean;
    /**
     * Option to draw additional information for joints
    */
    drawJointExtras: boolean;
    /**
     * Option to draw the bounding boxes for shapes
    */
    drawAABBs: boolean;
    /**
     * Option to draw the mass and center of mass of dynamic bodies
    */
    drawMass: boolean;
    /**
     * Option to draw contact points
    */
    drawContacts: boolean;
    /**
     * Option to visualize the graph coloring used for contacts and joints
    */
    drawGraphColors: boolean;
    /**
     * Option to draw contact normals
    */
    drawContactNormals: boolean;
    /**
     * Option to draw contact normal impulses
    */
    drawContactImpulses: boolean;
    /**
     * Option to draw contact friction impulses
    */
    drawFrictionImpulses: boolean;
    /**
     * User context that is passed as an argument to drawing callback functions
    */
    context: any;
}

/**
 * @property categoryBits - The collision category bits. Normally you would just set one bit.
 * The category bits should represent your application object types.
 * @property maskBits - The collision mask bits. States the categories that this shape would
 * accept for collision.
 * @property groupIndex - Collision groups allow a certain group of objects to never collide
 * (negative) or always collide (positive). Zero has no effect. Non-zero group filtering always wins
 * against the mask bits.
 */
declare class b2Filter {
    /**
     * The collision category bits. Normally you would just set one bit.
    The category bits should represent your application object types.
    */
    categoryBits: number;
    /**
     * The collision mask bits. States the categories that this shape would
    accept for collision.
    */
    maskBits: number;
    /**
     * Collision groups allow a certain group of objects to never collide
    (negative) or always collide (positive). Zero has no effect. Non-zero group filtering always wins
    against the mask bits.
    */
    groupIndex: number;
}

/**
 * @property categoryBits - The collision category bits of this query. Normally you would just set one bit.
 * @property maskBits - The collision mask bits. This states the shape categories that this query would accept for collision.
 */
declare class b2QueryFilter {
    /**
     * The collision category bits of this query. Normally you would just set one bit.
    */
    categoryBits: number;
    /**
     * The collision mask bits. This states the shape categories that this query would accept for collision.
    */
    maskBits: number;
}

/**
 * This file includes code that is:
 *
 * - Copyright 2023 Erin Catto, released under the MIT license.
 * - Copyright 2024 Phaser Studio Inc, released under the MIT license.
 */
declare const b2_maxWorkers: any;

/**
 * This function returns performance profiling data for a Box2D world.
 * Not supported in Phaser Box2D JS implementation.
 * @param worldId - The identifier of the Box2D world.
 * @returns A profile object containing performance metrics.
 */
declare function b2World_GetProfile(worldId: b2WorldId): b2Profile;

/**
 * This function is not supported in the Phaser Box2D JS implementation and will
 * generate a console warning when called.
 * @param worldId - The ID of the Box2D world instance.
 * @returns An object containing various Box2D performance counters.
 */
declare function b2World_GetCounters(worldId: b2WorldId): b2Counters;

/**
 * This function is a stub that displays a warning message indicating that memory statistics
 * dumping is not supported in the Phaser Box2D JavaScript implementation.
 * @param worldId - The identifier of the Box2D world
 */
declare function b2World_DumpMemoryStats(worldId: b2WorldId): void;

declare namespace Island { }

declare namespace Joint { }

/**
 * Creates and returns a new b2DistanceJointDef object initialized with specific default values.
 * The length is set to 1 unit and the maxLength is set to B2_HUGE. All other properties
 * of the joint definition retain their default values from the b2DistanceJointDef constructor.
 * @returns A distance joint definition with:
 * - length set to 1
 * - maxLength set to B2_HUGE
 * - all other properties at their default values
 */
declare function b2DefaultDistanceJointDef(): b2DistanceJointDef;

/**
 * Initializes a new b2MotorJointDef with common default values.
 * The joint definition includes preset values for maximum force,
 * maximum torque, and correction factor while using default
 * values for linear and angular offsets.
 * @returns A motor joint definition with:
 * - maxForce: 1
 * - maxTorque: 1
 * - correctionFactor: 0.3
 * - linearOffset: (0,0)
 * - angularOffset: 0
 */
declare function b2DefaultMotorJointDef(): b2MotorJointDef;

/**
 * Creates and returns a new b2MouseJointDef object initialized with default values
 * for frequency (hertz), damping ratio, and maximum force parameters.
 * @returns A mouse joint definition with:
 * - hertz = 4
 * - dampingRatio = 1
 * - maxForce = 1
 */
declare function b2DefaultMouseJointDef(): b2MouseJointDef;

/**
 * Creates and returns a new b2PrismaticJointDef instance with localAxisA initialized
 * to a unit vector pointing along the x-axis (1,0). All other properties retain their
 * default values from the b2PrismaticJointDef constructor.
 * @returns A prismatic joint definition with localAxisA set to (1,0)
 */
declare function b2DefaultPrismaticJointDef(): b2PrismaticJointDef;

/**
 * Creates and initializes a new b2RevoluteJointDef with default values.
 * Sets the drawSize property to 0.25 for visualization purposes.
 * @returns A new revolution joint definition with drawSize set to 0.25
 */
declare function b2DefaultRevoluteJointDef(): b2RevoluteJointDef;

/**
 * Creates and returns a new b2WeldJointDef object initialized with default values.
 * A weld joint essentially glues two bodies together at a reference point.
 * @returns A new weld joint definition with default configuration:
 * - localAnchorA: b2Vec2(0,0)
 * - localAnchorB: b2Vec2(0,0)
 * - referenceAngle: 0
 * - stiffness: 0
 * - damping: 0
 */
declare function b2DefaultWeldJointDef(): b2WeldJointDef;

/**
 * Creates and returns a new b2WheelJointDef with common default values.
 * The joint's local axis is set to point upward, and spring behavior
 * is enabled with standard frequency and damping values.
 * @returns A wheel joint definition with:
 * - localAxisA set to (0,1)
 * - enableSpring set to true
 * - hertz set to 1
 * - dampingRatio set to 0.7
 */
declare function b2DefaultWheelJointDef(): b2WheelJointDef;

/**
 * @param worldId - The ID of the Box2D world
 * @param def - The joint definition containing:
 * - bodyIdA: ID of the first body
 * - bodyIdB: ID of the second body
 * - localAnchorA: Local anchor point on body A
 * - localAnchorB: Local anchor point on body B
 * - length: Desired distance between anchor points
 * - minLength: Minimum allowed distance
 * - maxLength: Maximum allowed distance
 * - hertz: Spring frequency in Hz
 * - dampingRatio: Spring damping ratio
 * - maxMotorForce: Maximum motor force
 * - motorSpeed: Motor speed
 * - enableSpring: Enable/disable spring
 * - enableLimit: Enable/disable length limits
 * - enableMotor: Enable/disable motor
 * - collideConnected: Allow collision between connected bodies
 * - userData: User data
 * @returns The ID of the created distance joint
 */
declare function b2CreateDistanceJoint(worldId: b2WorldId, def: b2DistanceJointDef): b2JointId;

/**
 * @param worldId - The ID of the Box2D world
 * @param def - The joint definition containing:
 * - bodyIdA: ID of the first body
 * - bodyIdB: ID of the second body
 * - linearOffset: The target linear offset between bodies
 * - angularOffset: The target angular offset between bodies
 * - maxForce: Maximum force that can be applied
 * - maxTorque: Maximum torque that can be applied
 * - correctionFactor: Position correction factor in [0,1]
 * - collideConnected: Whether bodies can collide
 * - userData: User data
 * @returns The ID of the created motor joint
 */
declare function b2CreateMotorJoint(worldId: b2WorldId, def: b2MotorJointDef): b2JointId;

/**
 * Creates a mouse joint between two bodies at a specified target point. The joint
 * transforms the target point into local coordinates for both bodies and initializes
 * the joint properties including frequency, damping ratio, and maximum force.
 * @param worldId - The ID of the Box2D world where the joint will be created
 * @param def - The mouse joint definition containing:
 * - bodyIdA: ID of the first body
 * - bodyIdB: ID of the second body
 * - target: Target point in world coordinates
 * - hertz: Frequency in Hertz
 * - dampingRatio: Damping ratio
 * - maxForce: Maximum force
 * - userData: User data
 * - collideConnected: Whether connected bodies can collide
 * @returns The ID of the created mouse joint
 */
declare function b2CreateMouseJoint(worldId: b2WorldId, def: b2MouseJointDef): b2JointId;

/**
 * @param worldId - The ID of the Box2D world
 * @param def - The joint definition containing properties like:
 * - bodyIdA: ID of first body
 * - bodyIdB: ID of second body
 * - localAnchorA: Local anchor point on body A
 * - localAnchorB: Local anchor point on body B
 * - referenceAngle: Initial angle between bodies (clamped to [-π,π])
 * - hertz: Spring frequency
 * - dampingRatio: Spring damping ratio
 * - lowerAngle: Lower angle limit (clamped to [-π,π])
 * - upperAngle: Upper angle limit (clamped to [-π,π])
 * - maxMotorTorque: Maximum motor torque
 * - motorSpeed: Motor speed
 * - enableSpring: Enable spring behavior
 * - enableLimit: Enable angle limits
 * - enableMotor: Enable motor
 * - collideConnected: Allow collision between connected bodies
 * - userData: User data
 * - drawSize: Drawing size
 * @returns ID of the created revolute joint
 */
declare function b2CreateRevoluteJoint(worldId: b2WorldId, def: b2RevoluteJointDef): b2JointId;

/**
 * Creates a prismatic joint between two bodies in a Box2D world. A prismatic joint
 * constrains two bodies to move relative to each other along a specified axis.
 * @param worldId - The identifier of the Box2D world
 * @param def - The joint definition containing:
 * - bodyIdA: First body ID
 * - bodyIdB: Second body ID
 * - localAnchorA: Anchor point on body A in local coordinates
 * - localAnchorB: Anchor point on body B in local coordinates
 * - localAxisA: The axis of translation in body A's local coordinates
 * - referenceAngle: The initial angle between the bodies
 * - enableLimit: Whether translation limits are enabled
 * - lowerTranslation: Lower translation limit
 * - upperTranslation: Upper translation limit
 * - enableMotor: Whether the motor is enabled
 * - motorSpeed: Motor speed
 * - maxMotorForce: Maximum motor force
 * - enableSpring: Whether spring is enabled
 * - hertz: Spring frequency in Hz
 * - dampingRatio: Spring damping ratio
 * - collideConnected: Whether connected bodies can collide
 * - userData: User data
 * @returns The identifier for the created prismatic joint
 */
declare function b2CreatePrismaticJoint(worldId: b2WorldId, def: b2PrismaticJointDef): b2JointId;

/**
 * Creates a weld joint between two bodies in a Box2D world.
 * @param worldId - The identifier of the Box2D world
 * @param def - The weld joint definition containing:
 * - bodyIdA: ID of the first body
 * - bodyIdB: ID of the second body
 * - localAnchorA: Local anchor point on body A
 * - localAnchorB: Local anchor point on body B
 * - referenceAngle: Reference angle between the bodies
 * - linearHertz: Frequency for the linear constraint
 * - linearDampingRatio: Damping ratio for the linear constraint
 * - angularHertz: Frequency for the angular constraint
 * - angularDampingRatio: Damping ratio for the angular constraint
 * - collideConnected: Whether the connected bodies can collide
 * - userData: User data associated with the joint
 * @returns The identifier of the created weld joint
 */
declare function b2CreateWeldJoint(worldId: b2WorldId, def: b2WeldJointDef): b2JointId;

/**
 * Creates a wheel joint between two bodies in a Box2D world. A wheel joint provides two degrees of
 * freedom: translation along a specified axis and rotation about an orthogonal axis. The joint can
 * be configured with a spring and damper mechanism, translation limits, and a motor.
 * @param worldId - The identifier of the Box2D world
 * @param def - The wheel joint definition containing configuration parameters
 * @returns The identifier of the created wheel joint
 */
declare function b2CreateWheelJoint(worldId: b2WorldId, def: b2WheelJointDef): b2JointId;

/**
 * Destroys a joint in the physics world. If the world is locked (e.g. during collision detection
 * or integration), the function will return without destroying the joint. The function internally
 * calls b2DestroyJointInternal to handle the actual joint destruction.
 * @param jointId - The identifier of the joint to be destroyed
 */
declare function b2DestroyJoint(jointId: b2JointId): void;

/**
 * Retrieves the joint type from the world using the provided joint ID.
 * The function first gets the world reference from the joint ID,
 * then retrieves the full joint object to access its type property.
 * @param jointId - The ID of the joint to query.
 * @returns The type of the specified joint.
 */
declare function b2Joint_GetType(jointId: b2JointId): b2JointType;

/**
 * This function retrieves the identifier of the first body (body A) that is
 * connected to the specified joint. The joint must exist in the world referenced
 * by the jointId.
 * @param jointId - The identifier of the joint.
 * @returns The identifier of the first body connected to the joint.
 */
declare function b2Joint_GetBodyA(jointId: b2JointId): b2BodyId;

/**
 * This function retrieves the identifier of the second body (body B) that is
 * connected to the specified joint. The joint must exist in the world referenced
 * by the jointId.
 * @param jointId - The identifier of the joint.
 * @returns The identifier of body B connected to the joint.
 */
declare function b2Joint_GetBodyB(jointId: b2JointId): b2BodyId;

/**
 * Retrieves the local anchor point A from a joint's simulation data. The anchor point
 * is expressed in the local coordinate system of body A.
 * @param jointId - The identifier for the joint.
 * @returns The local anchor point A in the body A frame.
 */
declare function b2Joint_GetLocalAnchorA(jointId: b2JointId): b2Vec2;

/**
 * Retrieves the local anchor point B of a joint, which represents the connection
 * point on the second body (body B) in that body's local coordinate system.
 * @param jointId - The identifier for the joint.
 * @returns The local anchor point B in the body's local coordinates.
 */
declare function b2Joint_GetLocalAnchorB(jointId: b2JointId): b2Vec2;

/**
 * When enabled, the bodies connected by the joint can collide with each other.
 * When disabled, collisions between the connected bodies are filtered out.
 * The function updates the broadphase when enabling collisions and destroys
 * existing contacts between the bodies when disabling collisions.
 * @param jointId - The identifier for the joint to modify.
 * @param shouldCollide - True to enable collision between connected bodies, false to disable.
 */
declare function b2Joint_SetCollideConnected(jointId: b2JointId, shouldCollide: boolean): void;

/**
 * Gets the collideConnected flag for the specified joint. This flag determines if
 * the two bodies connected by this joint are allowed to collide with each other.
 * @param jointId - The ID of the joint to query
 * @returns Whether the connected bodies can collide with each other
 */
declare function b2Joint_GetCollideConnected(jointId: b2JointId): boolean;

/**
 * Associates arbitrary user data with a specified joint in the physics world.
 * The joint is located using its world and joint identifiers, and its user data
 * property is updated with the provided value.
 * @param jointId - The identifier for the joint to modify.
 * @param userData - The user data to associate with the joint.
 */
declare function b2Joint_SetUserData(jointId: b2JointId, userData: any): void;

/**
 * Retrieves the user data that was previously attached to the specified joint.
 * The function first gets the world object from the joint ID, then retrieves
 * the joint using the full joint ID, and finally returns the userData property
 * of that joint.
 * @param jointId - The identifier for the joint.
 * @returns The user data associated with the joint.
 */
declare function b2Joint_GetUserData(jointId: b2JointId): void;

/**
 * Wakes up both bodies connected by a joint in the physics simulation.
 * @param jointId - The identifier for the joint whose connected bodies should be awakened.
 */
declare function b2Joint_WakeBodies(jointId: b2JointId): void;

/**
 * Returns the constraint force for different joint types including distance, motor,
 * mouse, prismatic, revolute, weld, and wheel joints. The force is retrieved from
 * the corresponding joint-specific force getter function.
 * @param jointId - The identifier for the joint.
 * @returns The constraint force vector. Returns (0,0) for unknown joint types.
 */
declare function b2Joint_GetConstraintForce(jointId: b2JointId): b2Vec2;

/**
 * Returns the constraint torque for different joint types including motor, mouse, prismatic,
 * revolute, weld and wheel joints. For distance joints, it always returns 0.
 * @param jointId - The ID of the joint to get the constraint torque from.
 * @returns The constraint torque value. Returns 0 for distance joints or if joint type is invalid.
 */
declare function b2Joint_GetConstraintTorque(jointId: b2JointId): number;

declare namespace Manifold { }

/**
 * Computes collision information between two circles transformed by their respective transforms.
 * @param circleA - The first circle
 * @param xfA - Transform for the first circle
 * @param circleB - The second circle
 * @param xfB - Transform for the second circle
 * @param manifold - The output collision manifold
 * @returns The collision manifold containing contact points, normal, and separation
 * information. If no collision is detected, returns a cleared manifold.
 */
declare function b2CollideCircles(circleA: b2Circle, xfA: b2Transform, circleB: b2Circle, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes collision information between a capsule shape and a circle shape.
 * @param capsuleA - The capsule shape A with properties center1, center2, and radius
 * @param xfA - Transform for shape A containing position (p) and rotation (q)
 * @param circleB - The circle shape B with properties center and radius
 * @param xfB - Transform for shape B containing position (p) and rotation (q)
 * @param manifold - Output collision manifold to be populated
 * @returns The populated collision manifold containing contact points,
 * normal, separation, and other collision data
 */
declare function b2CollideCapsuleAndCircle(capsuleA: b2Capsule, xfA: b2Transform, circleB: b2Circle, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes collision information between a polygon and a circle.
 * @param polygonA - The polygon shape
 * @param xfA - Transform for polygon A
 * @param circleB - The circle shape
 * @param xfB - Transform for circle B
 * @param manifold - Output collision manifold
 * @returns The collision manifold containing contact points, normal, and separation
 */
declare function b2CollidePolygonAndCircle(polygonA: b2Polygon, xfA: b2Transform, circleB: b2Circle, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes the collision manifold between two capsule shapes in 2D space.
 * A capsule is defined by two endpoints and a radius.
 * @param capsuleA - The first capsule shape
 * @param xfA - Transform for the first capsule
 * @param capsuleB - The second capsule shape
 * @param xfB - Transform for the second capsule
 * @param manifold - Output collision manifold
 * @returns Modifies the manifold parameter with collision data:
 * - normalX/Y: Collision normal vector
 * - pointCount: Number of contact points (0-2)
 * - points[]: Contact point data including:
 * - anchorA/B: Contact points in local coordinates
 * - separation: Penetration depth
 * - id: Contact ID
 */
declare function b2CollideCapsules(capsuleA: b2Capsule, xfA: b2Transform, capsuleB: b2Capsule, xfB: b2Transform, manifold: b2Manifold): void;

/**
 * Converts the segment to a zero-radius capsule and delegates to b2CollideCapsules
 * for the actual collision computation.
 * @param segmentA - A line segment defined by two points (point1, point2)
 * @param xfA - Transform for segmentA containing position and rotation
 * @param capsuleB - A capsule shape defined by two points and a radius
 * @param xfB - Transform for capsuleB containing position and rotation
 * @returns Collision manifold containing contact points and normal
 */
declare function b2CollideSegmentAndCapsule(segmentA: b2Segment, xfA: b2Transform, capsuleB: b2Capsule, xfB: b2Transform): b2Manifold;

/**
 * Computes collision manifold between a polygon and a capsule by converting the capsule
 * to a polygon and using polygon-polygon collision detection.
 * @param polygonA - The first collision shape (polygon)
 * @param xfA - Transform for polygon A, containing position and rotation
 * @param capsuleB - The second collision shape (capsule) defined by two centers and a radius
 * @param xfB - Transform for capsule B, containing position and rotation
 * @param manifold - The output collision manifold to be populated
 * @returns The collision manifold containing contact points and normal
 */
declare function b2CollidePolygonAndCapsule(polygonA: b2Polygon, xfA: b2Transform, capsuleB: b2Capsule, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Detects collision between two polygons and generates contact information.
 * Transforms the polygons into a common frame, finds the separating axes,
 * and generates contact points. The manifold includes contact points with
 * anchor positions, separation distance, contact IDs and collision normal.
 * @param polygonA - First polygon
 * @param xfA - Transform for first polygon containing position (p) and rotation (q)
 * @param polygonB - Second polygon
 * @param xfB - Transform for second polygon containing position (p) and rotation (q)
 * @param manifold - Output manifold to store collision data
 * @returns The collision manifold containing contact points, normal and separation
 */
declare function b2CollidePolygons(polygonA: b2Polygon, xfA: b2Transform, polygonB: b2Polygon, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes collision detection between a line segment and a circle by converting
 * the segment to a zero-radius capsule and using capsule-circle collision.
 * @param segmentA - The line segment shape
 * @param xfA - Transform for segmentA
 * @param circleB - The circle shape
 * @param xfB - Transform for circleB
 * @param manifold - The collision manifold to populate
 * @returns The collision manifold containing contact information
 */
declare function b2CollideSegmentAndCircle(segmentA: b2Segment, xfA: b2Transform, circleB: b2Circle, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes collision manifold between a line segment and a polygon by converting
 * the segment into a zero-width capsule and using polygon collision detection.
 * @param segmentA - The line segment
 * @param xfA - Transform for segmentA
 * @param polygonB - The polygon to test collision against
 * @param xfB - Transform for polygonB
 * @param manifold - The manifold to populate with collision data
 * @returns Collision manifold containing contact points and normal
 */
declare function b2CollideSegmentAndPolygon(segmentA: b2Segment, xfA: b2Transform, polygonB: b2Polygon, xfB: b2Transform, manifold: b2Manifold): b2Manifold;

/**
 * Computes collision detection between a chain segment and a circle.
 * @param chainSegmentA - A chain segment with properties segment (containing point1 and point2) and ghost points (ghost1, ghost2)
 * @param xfA - Transform for chain segment containing position (p) and rotation (q)
 * @param circleB - Circle object with center point and radius
 * @param xfB - Transform for circle containing position (p) and rotation (q)
 * @param manifold - Contact manifold to store collision results
 * @returns The manifold object containing collision data:
 * - normalX/Y: collision normal in world coordinates
 * - points: array with contact point data including:
 * - anchorA/B: contact points in local coordinates
 * - point: contact point in world coordinates
 * - separation: distance between shapes
 * - id: contact ID
 * - pointCount: number of contact points
 */
declare function b2CollideChainSegmentAndCircle(chainSegmentA: any, xfA: any, circleB: any, xfB: any, manifold: any): any;

/**
 * Computes collision between a chain segment and a capsule by converting the capsule
 * to a polygon and delegating to the segment-polygon collision function.
 * @param segmentA - The chain segment shape
 * @param xfA - Transform for segmentA
 * @param capsuleB - The capsule shape defined by two centers and a radius
 * @param xfB - Transform for capsuleB
 * @param cache - Simplex cache for persistent contact information
 * @param manifold - Contact manifold to store collision results
 */
declare function b2CollideChainSegmentAndCapsule(segmentA: b2ChainSegment, xfA: b2Transform, capsuleB: b2Capsule, xfB: b2Transform, cache: b2SimplexCache, manifold: b2Manifold): void;

/**
 * Computes the collision manifold between a chain segment (a segment with rounded corners)
 * and a polygon. The function handles edge cases including convex/concave corners and
 * determines contact points and normals. The manifold is populated with contact points
 * and can contain 0, 1 or 2 contact points depending on the collision configuration.
 * @param chainSegmentA - The chain segment shape A
 * @param xfA - Transform for shape A
 * @param polygonB - The polygon shape B
 * @param xfB - Transform for shape B
 * @param cache - Cache for distance calculations
 * @param manifold - The contact manifold to populate
 * @returns The populated contact manifold
 */
declare function b2CollideChainSegmentAndPolygon(chainSegmentA: b2ChainSegment, xfA: b2Transform, polygonB: b2Polygon, xfB: b2Transform, cache: b2DistanceCache, manifold: b2Manifold): b2Manifold;

declare namespace MathFunctions { }

/**
 * This function performs a validation check on a number by ensuring it is both
 * finite and not NaN (Not a Number).
 * @param a - The number to validate.
 * @returns True if the number is valid (finite and not NaN), false otherwise.
 */
declare function b2IsValid(a: number): boolean;

/**
 * Validates a b2Vec2 object by checking if it exists and its components are valid numbers.
 * @param v - The vector to validate, containing x and y components.
 * @returns True if the vector exists and both x and y components are valid numbers.
 */
declare function b2Vec2_IsValid(v: b2Vec2): boolean;

/**
 * Checks if a b2Rot object is valid by verifying:
 * 1. The object exists
 * 2. Both sine and cosine components contain valid numbers
 * 3. The rotation is properly normalized (s² + c² = 1)
 * @param q - A rotation object containing sine (s) and cosine (c) components
 * @returns True if the rotation is valid, false otherwise
 */
declare function b2Rot_IsValid(q: b2Rot): boolean;

/**
 * Normalizes the input vector by dividing its components by its length.
 * If the vector's length is greater than the epsilon value, the function
 * returns a new vector with the same direction but unit length.
 * @param v - The vector to normalize.
 * @returns Returns a new normalized b2Vec2 if successful.
 * If the vector length is less than epsilon, returns a zero vector (0,0).
 */
declare function b2Normalize(v: b2Vec2): b2Vec2;

/**
 * Normalizes a 2D vector and performs length validation.
 * @param v - The vector to normalize.
 * @returns A new normalized vector with unit length.
 */
declare function b2NormalizeChecked(v: b2Vec2): b2Vec2;

/**
 * Calculates the length of a vector and returns its normalized form.
 * @param v - The input vector to normalize
 * @returns An object containing:
 * - length: The original length of the vector
 * - normal: A normalized vector (unit length) in the same direction as v.
 * Returns (0,0) if the input vector length is below epsilon
 */
declare function b2GetLengthAndNormalize(v: b2Vec2): any;

declare namespace MotorJoint { }

/**
 * Updates the target linear offset of a motor joint. The linear offset represents
 * the desired translation between the two bodies connected by the joint.
 * @param jointId - The identifier for the motor joint to modify.
 * @param linearOffset - The desired linear offset in local coordinates.
 */
declare function b2MotorJoint_SetLinearOffset(jointId: b2JointId, linearOffset: b2Vec2): void;

/**
 * Gets the linear offset of a motor joint.
 * @param jointId - The identifier of the motor joint.
 * @returns The linear offset vector of the motor joint.
 */
declare function b2MotorJoint_GetLinearOffset(jointId: b2JointId): b2Vec2;

/**
 * Sets the target angular offset for a motor joint, which defines the desired relative rotation
 * between the connected bodies. The input angle is automatically clamped to the range [-π, π].
 * @param jointId - The identifier for the motor joint to modify.
 * @param angularOffset - The desired angular offset in radians, clamped between -π and π.
 */
declare function b2MotorJoint_SetAngularOffset(jointId: b2JointId, angularOffset: number): void;

/**
 * @param jointId - The identifier of the motor joint.
 * @returns The angular offset value of the motor joint in radians.
 */
declare function b2MotorJoint_GetAngularOffset(jointId: b2JointId): number;

/**
 * Sets the maximum force that can be applied by a motor joint.
 * @param jointId - The identifier for the motor joint
 * @param maxForce - The maximum force value to set. Will be clamped to non-negative values.
 */
declare function b2MotorJoint_SetMaxForce(jointId: b2JointId, maxForce: number): void;

/**
 * Gets the maximum force value from a motor joint.
 * @param jointId - The identifier for the motor joint.
 * @returns The maximum force value of the motor joint.
 */
declare function b2MotorJoint_GetMaxForce(jointId: b2JointId): number;

/**
 * Sets the maximum torque that can be applied by a motor joint.
 * @param jointId - The identifier for the motor joint.
 * @param maxTorque - The maximum torque value. Will be clamped to non-negative values.
 */
declare function b2MotorJoint_SetMaxTorque(jointId: b2JointId, maxTorque: number): void;

/**
 * Gets the maximum torque value for a motor joint.
 * @param jointId - The identifier for the motor joint.
 * @returns The maximum torque value of the motor joint.
 */
declare function b2MotorJoint_GetMaxTorque(jointId: b2JointId): number;

/**
 * Sets the position correction factor for a motor joint, which determines how much position error is corrected each time step.
 * The correction factor is automatically clamped between 0 and 1.
 * @param jointId - The identifier for the motor joint.
 * @param correctionFactor - The correction factor value, clamped between 0 and 1.
 */
declare function b2MotorJoint_SetCorrectionFactor(jointId: b2JointId, correctionFactor: number): void;

/**
 * Gets the correction factor of a motor joint.
 * @param jointId - The identifier for the motor joint.
 * @returns The correction factor value of the motor joint.
 */
declare function b2MotorJoint_GetCorrectionFactor(jointId: b2JointId): number;

declare namespace MouseJoint { }

/**
 * Updates the target position of a mouse joint by cloning the provided target vector.
 * The joint must be of type b2_mouseJoint.
 * @param jointId - The identifier of the mouse joint to modify.
 * @param target - The new target position vector to set.
 */
declare function b2MouseJoint_SetTarget(jointId: b2JointId, target: b2Vec2): void;

/**
 * @param jointId - The identifier of the mouse joint.
 * @returns The target point of the mouse joint.
 */
declare function b2MouseJoint_GetTarget(jointId: b2JointId): b2Vec2;

/**
 * @param jointId - The identifier for the mouse joint to modify.
 * @param hertz - The spring frequency in Hertz (Hz).
 */
declare function b2MouseJoint_SetSpringHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the spring frequency in Hertz from a mouse joint.
 * @param jointId - The identifier for the mouse joint.
 * @returns The spring frequency in Hertz.
 */
declare function b2MouseJoint_GetSpringHertz(jointId: b2JointId): number;

/**
 * Sets the damping ratio for a mouse joint.
 * @param jointId - The identifier for the mouse joint to modify.
 * @param dampingRatio - The damping ratio value to set.
 */
declare function b2MouseJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the spring damping ratio of a mouse joint.
 * @param jointId - The identifier of the mouse joint.
 * @returns The spring damping ratio value of the mouse joint.
 */
declare function b2MouseJoint_GetSpringDampingRatio(jointId: b2JointId): number;

/**
 * Sets the maximum force that can be applied by the mouse joint to maintain its constraint.
 * @param jointId - The identifier for the mouse joint to modify.
 * @param maxForce - The maximum force value to set for the joint.
 */
declare function b2MouseJoint_SetMaxForce(jointId: b2JointId, maxForce: number): void;

/**
 * Gets the maximum force value from a mouse joint.
 * @param jointId - The identifier for the mouse joint.
 * @returns The maximum force value of the mouse joint.
 */
declare function b2MouseJoint_GetMaxForce(jointId: b2JointId): number;

declare namespace Physics {
    /**
     * Creates a box-shaped polygon and attaches it to a body based on the dimensions, position
     * and rotation of the given Sprite.
     * @param worldId - The World ID.
     * @param sprite - The Sprite object to read the data from.
     * @param data - Additional configuration for the box polygon.
     * @returns The created box's body ID, shape ID, and object.
     */
    function SpriteToBox(worldId: number, sprite: Sprite, data: BoxPolygonConfig): any;
    /**
     * Creates a circle-shaped polygon and attaches it to a body based on the dimensions, position
     * and rotation of the given Sprite.
     * @param worldId - The World ID.
     * @param sprite - The Sprite object to read the data from.
     * @param data - Additional configuration for the circle.
     * @returns The created box's body ID, shape ID, and object.
     */
    function SpriteToCircle(worldId: number, sprite: Sprite, data: CircleConfig): any;
    /**
     * Creates a world and returns the ID.
     * @param data - Configuration for the world.
     * @returns The created world's ID.
     */
    function CreateWorld(data: WorldConfig): any;
    /**
     * Creates a chain of capsules with each one linked to the previous and next one.
     * @param data - Configuration for the chain.
     * @returns A list of each link's body ID, shape ID, and object.
     */
    function CreateChain(data: ChainConfig): BodyCapsule[];
    /**
     * Creates a circle shape and attaches it to a body.
     * @param data - Configuration for the circle.
     * @returns The created circle's body ID, shape ID, and object.
     */
    function CreateCircle(data: CircleConfig): any;
    /**
     * Creates a capsule shape and attaches it to a body.
     * @param data - Configuration for the capsule.
     * @returns The created capsule's body ID, shape ID, and object.
     */
    function CreateCapsule(data: CapsuleConfig): BodyCapsule;
    /**
     * Creates a box-shaped polygon and attaches it to a body.
     * @param data - Configuration for the box polygon.
     * @returns The created box's body ID, shape ID, and object.
     */
    function CreateBoxPolygon(data: BoxPolygonConfig): any;
    /**
     * Creates a regular n-gon polygon and attaches it to a body.
     * @param data - Configuration for the n-gon polygon.
     * @returns The created n-gon's body ID, shape ID, and object.
     */
    function CreateNGonPolygon(data: NGonPolygonConfig): any;
    /**
     * Creates a polygon and attaches it to a body.
     * @param data - Configuration for the polygon.
     * @returns The created polygon's body ID, shape ID, and object.
     */
    function CreatePolygon(data: PolygonConfig): any;
    /**
     * Creates a polygon from Earcut CDT data and attaches it to a body.
     * @param data - Configuration for the polygon.
     * @returns The created polygon's body ID, shape ID, and object.
     */
    function CreatePolygonFromEarcut(data: PolygonVertexConfig): any;
    /**
     * Creates a polygon from Vertex and Index data and attaches it to a body.
     * @param data - Configuration for the polygon.
     * @returns The created polygon's body ID, shape ID, and object.
     */
    function CreatePolygonFromVertices(data: PolygonVertexConfig): any;
    /**
     * Creates a polygon from PhysicsEditor XML data and attaches it to a body.
     * It is recommended to prepare data with this _before_ the game loop starts; It is async and quite slow.
     * @param data - Configuration for the polygon.
     * @returns The created polygon's body ID, shape ID, and object.
     */
    function CreatePhysicsEditorShape(data: PolygonVertexConfig): Promise<{ bodyId: b2BodyId; shapeId: b2ShapeId; object: b2Polygon; }>;
    /**
     * Creates a revolute joint between two bodies.
     * @param data - Configuration for the revolute joint.
     * @returns The ID of the created revolute joint.
     */
    function CreateRevoluteJoint(data: RevoluteJointConfig): any;
    /**
     * Creates a weld joint between two bodies.
     * @param data - Configuration for the weld joint.
     * @returns The ID of the created weld joint.
     */
    function CreateWeldJoint(data: WeldJointConfig): any;
    /**
     * Creates a distance joint between two bodies.
     * @param data - Configuration for the distance joint.
     * @returns The ID of the created distance joint.
     */
    function CreateDistanceJoint(data: DistanceJointConfig): any;
    /**
     * Creates a wheel joint between two bodies.
     * @param data - Configuration for the wheel joint.
     * @returns The ID of the created wheel joint.
     */
    function CreateWheelJoint(data: WheelJointConfig): any;
    /**
     * Creates a prismatic joint between two bodies.
     * @param data - Configuration for the prismatic joint.
     * @returns The ID of the created prismatic joint.
     */
    function CreatePrismaticJoint(data: PrismaticJointConfig): any;
    /**
     * Creates a motor joint between two bodies.
     * @param data - Configuration for the motor joint.
     * @returns The ID of the created motor joint.
     */
    function CreateMotorJoint(data: MotorJointConfig): any;
    /**
     * Creates a mouse joint between two bodies.
     * @param data - Configuration for the mouse joint.
     * @returns The ID of the created mouse joint.
     */
    function CreateMouseJoint(data: MouseJointConfig): any;
}

/**
 * Set the scale of the Box2D World when converting from pixels to meters.
 */
declare function SetWorldScale(scale: number): void;

/**
 * Get the current scale of the Box2D World, as used when converting from pixels to meters.
 */
declare function GetWorldScale(): number;

/**
 * Converts a single numerical value from meters to pixels.
 */
declare function mpx(meters: number): number;

/**
 * Converts a single numerical value from pixels to meters.
 */
declare function pxm(pixels: number): number;

/**
 * Converts the given x and y values from pixels to meters, stored in a new b2Vec2.
 */
declare function pxmVec2(x: number, y: number): b2Vec2;

/**
 * Convets the given value in radians to a b2Rot object, used for Box2D rotations.
 */
declare function RotFromRad(radians: number): b2Rot;

/**
 * Adds a Sprite Game Object to the given World, attaching it to the given Body.
 */
declare function AddSpriteToWorld(worldId: number, sprite: Sprite, body: b2Body): void;

/**
 * Removes a Sprite Game Object from the given World, optionally destroying the Body it was attached to.
 */
declare function RemoveSpriteFromWorld(worldId: number, sprite: Sprite, destroyBody?: boolean): void;

/**
 * Clears all Sprite to Body pairs.
 * Neither the Sprites nor the Bodies are destroyed.
 * The bodies remain in the world.
 */
declare function ClearWorldSprites(worldId: number): void;

/**
 * Returns the Body attached to the given Sprite in the given World.
 * Or `null` if no such pair exists.
 * @returns Either the sprite, or `null`.
 */
declare function GetBodyFromSprite(worldId: number, sprite: Sprite): Sprite | null;

/**
 * Runs through all World-Sprite pairs and updates the Sprite positions and rotations to match the Body.
 */
declare function UpdateWorldSprites(worldId: number): void;

/**
 * Converts a Box2D Body's position and rotation to a Sprite's position and rotation.
 *
 * This is called automatically by `UpdateWorldSprites`.
 */
declare function BodyToSprite(body: b2Body, sprite: Sprite): void;

/**
 * @property x - The x position of the sprite.
 * @property y - The y position of the sprite.
 * @property width - The width of the sprite.
 * @property height - The height of the sprite.
 * @property rotation - The rotation of the sprite in radians.
 * @property [scaleX] - Optional horizontal scale of the sprite.
 * @property [scaleY] - Optional vertical scale of the sprite.
 * @property [scale] - Optional scale vector of the sprite.
 */
declare type Sprite = {
    x: number;
    y: number;
    width: number;
    height: number;
    rotation: number;
    scaleX?: number;
    scaleY?: number;
    scale?: b2Vec2;
};

/**
 * @property [worldDef] - World definition
 */
declare type WorldConfig = {
    worldDef?: b2WorldDef;
};

/**
 * @property worldId - The world ID value
 * @property deltaTime - How long has it been since the last call (e.g. the value passed to a RAF update)
 * @property [fixedTimeStep = 1/60] - Duration of the fixed timestep for the Physics simulation
 * @property [subStepCount = 4] - Number of sub-steps performed per world step
 */
declare type WorldStepConfig = {
    worldId: b2WorldId;
    deltaTime: number;
    fixedTimeStep?: number;
    subStepCount?: number;
};

/**
 * Steps a physics world to match fixedTimeStep.
 * Returns the average time spent in the step function.
 * @param data - Configuration for the world.
 * @returns totalTime - Time spent processing the step function, in seconds.
 */
declare function WorldStep(data: WorldConfig): number;

/**
 * @property worldId - ID for the world.
 * @property groundId - ID for the static ground to attach the chain ends to.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property firstLinkPosition - Position of the first link.
 * @property lastLinkPosition - Position of the last link.
 * @property chainLinks - Number of links in the chain.
 * @property linkLength - Length of each link
 * @property [density] - Density of the links.
 * @property [friction] - Friction of the links.
 * @property [color] - Custom color for the links.
 * @property [radius] - Radius for the circle 'caps' on each capsule link.
 * @property fixEnds - Should the ends of the chain be fixed to the groundId object?
 */
declare type ChainConfig = {
    worldId: b2WorldId;
    groundId: b2BodyId;
    type?: number;
    firstLinkPosition: b2Vec2;
    lastLinkPosition: b2Vec2;
    chainLinks: number;
    linkLength: number;
    density?: number;
    friction?: number;
    color?: any;
    radius?: number;
    fixEnds: boolean;
};

/**
 * @property bodyId - ID for the body to attach the capsule to.
 * @property shapeId - ID for the shape to attach the capsule to.
 */
declare type BodyCapsule = {
    bodyId: b2BodyId;
    shapeId: b2ShapeId;
};

/**
 * @property worldId - ID for the world in which to create the circle.
 * @property [bodyDef] - Body definition for the circle.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the circle's center.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the circle.
 * @property [groupIndex] - Collision filtering, group index for the circle.
 * @property [categoryBits] - Collision filtering, what 'category' is this cirle in?
 * @property [maskBits] - Collision filtering, what 'categories' will this circle collide with?
 * @property [density] - Density of the circle.
 * @property [friction] - Friction of the circle.
 * @property [restitution = 0.1] - Restitution of the circle.
 * @property [color] - Custom color for the circle.
 * @property [radius] - Radius of the circle.
 * @property [preSolve] - Enable presolve callback for the circle.
 * @property [isSensor] - A sensor shape generates overlap events but never generates a collision response
 * @property [offset] - Offset of the circle's center when adding as a fixture.
 */
declare type CircleConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    groupIndex?: number;
    categoryBits?: number;
    maskBits?: number;
    density?: number;
    friction?: number;
    restitution?: number;
    color?: any;
    radius?: number;
    preSolve?: boolean;
    isSensor?: boolean;
    offset?: b2Vec2;
};

/**
 * @property worldId - ID for the world in which to create the capsule.
 * @property [bodyDef] - Body definition for the capsule.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the capsule's center.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the capsule.
 * @property [density] - Density of the capsule.
 * @property [friction] - Friction of the capsule.
 * @property [groupIndex] - Collision group index for the capsule.
 * @property [categoryBits] - Collision filtering, what 'category' is this in?
 * @property [maskBits] - Collision filtering, what 'categories' will this collide with?
 * @property [color] - Custom color for the capsule.
 * @property [center1] - Center of the first circle of the capsule. Optional if 'height' is set.
 * @property [center2] - Center of the second circle of the capsule. Optional if 'height' is set.
 * @property [radius] - Radius of the capsule's circles. Optional if 'width' is set.
 * @property [fixedRotation] - Prevent rotation if set to true.
 * @property [linearDamping] - Linear damping of velocity.
 * @property [width] - The overall width of the capsule. If set replaces radius.
 * @property [height] - The overall height of the capsule, including the start and end caps. If set replaces center1 and center2.
 */
declare type CapsuleConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    density?: number;
    friction?: number;
    groupIndex?: number;
    categoryBits?: number;
    maskBits?: number;
    color?: any;
    center1?: b2Vec2;
    center2?: b2Vec2;
    radius?: number;
    fixedRotation?: boolean;
    linearDamping?: number;
    width?: number;
    height?: number;
};

/**
 * @property worldId - ID for the world in which to create the box.
 * @property [bodyDef] - Body definition for the box.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the box's center.
 * @property [fixedRotation] - Prevent box from rotating?
 * @property [linearDamping] - Damping for linear velocity.
 * @property [angularDamping] - Damping for angular velocity.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the box.
 * @property [userData] - The user data to associate with the body.
 * @property [groupIndex] - Collision group index for the box.
 * @property [categoryBits] - Collision filtering, what 'category' is this in?
 * @property [maskBits] - Collision filtering, what 'categories' will this collide with?
 * @property [density = 1.0] - Density of the box.
 * @property [friction = 0.6] - Friction of the box.
 * @property [restitution = 0.1] - Restitution of the box.
 * @property [color] - Custom color for the box.
 * @property [preSolve] - Enable presolve callback for the circle.
 * @property size - Size of the box (either a b2Vec2 or a single number for square).
 */
declare type BoxPolygonConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    fixedRotation?: boolean;
    linearDamping?: number;
    angularDamping?: number;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    userData?: any;
    groupIndex?: number;
    categoryBits?: number;
    maskBits?: number;
    density?: number;
    friction?: number;
    restitution?: number;
    color?: any;
    preSolve?: boolean;
    size: b2Vec2 | number;
};

/**
 * @property worldId - ID for the world in which to create the n-gon.
 * @property [bodyDef] - Body definition for the n-gon.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the n-gon's center.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the n-gon.
 * @property [groupIndex] - Collision group index for the n-gon.
 * @property [density] - Density of the n-gon.
 * @property [friction] - Friction of the n-gon.
 * @property [color] - Custom color for the n-gon.
 * @property radius - Radius of the n-gon.
 * @property sides - Number of sides for the n-gon.
 */
declare type NGonPolygonConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    groupIndex?: number;
    density?: number;
    friction?: number;
    color?: any;
    radius: number;
    sides: number;
};

/**
 * @property worldId - ID for the world in which to create the polygon.
 * @property [bodyDef] - Body definition for the polygon.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the polygon's center.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the polygon.
 * @property [groupIndex] - Collision group index for the polygon.
 * @property [density] - Density of the polygon.
 * @property [friction] - Friction of the polygon.
 * @property [color] - Custom color for the polygon.
 * @property vertices - List of vertices for the polygon.
 */
declare type PolygonConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    groupIndex?: number;
    density?: number;
    friction?: number;
    color?: any;
    vertices: b2Vec2[];
};

/**
 * @property worldId - ID for the world in which to create the polygon.
 * @property [bodyDef] - Body definition for the polygon.
 * @property [type] - Type of the body (static, dynamic, kinematic).
 * @property [position] - Position of the polygon's center.
 * @property [bodyId] - Existing body ID if adding as a fixture.
 * @property [shapeDef] - Shape definition for the polygon.
 * @property [groupIndex] - Collision group index for the polygon.
 * @property [density] - Density of the polygon.
 * @property [friction] - Friction of the polygon.
 * @property [restitution = 0.1] - Restitution of the polygon.
 * @property [color] - Custom color for the polygon.
 * @property indices - List of indices to the vertices for the polygon.
 * @property vertices - List of vertices for the polygon in number pairs [x0,y0, x1,y1, ... xN,yN].
 * @property vertexOffset - Offset to recenter the vertices if they are not zero based.
 * @property vertexScale - Scale for the vertices, defaults to 1, 1.
 * @property [url] - URL location of the XML data file, if we're using one.
 * @property [key] - Name 'key' to find the correct data in the XML.
 */
declare type PolygonVertexConfig = {
    worldId: b2WorldId;
    bodyDef?: b2BodyDef;
    type?: number;
    position?: b2Vec2;
    bodyId?: b2BodyId;
    shapeDef?: b2ShapeDef;
    groupIndex?: number;
    density?: number;
    friction?: number;
    restitution?: number;
    color?: any;
    indices: number[];
    vertices: number[];
    vertexOffset: b2Vec2;
    vertexScale: b2Vec2;
    url?: string;
    key?: string;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2RevoluteJointDef.
 * @property bodyIdA - The first body to connect with this joint.
 * @property bodyIdB - The second body to connect with this joint.
 * @property [anchorA] - Local position of the anchor point on the first body.
 * @property [anchorB] - Local position of the anchor point on the second body.
 * @property [lowerAngle] - Lower limit of the joint's angle.
 * @property [upperAngle] - Upper limit of the joint's angle.
 * @property [enableLimit] - Whether to enable angle limits.
 * @property [enableMotor] - Whether to enable the joint's motor.
 * @property [motorSpeed] - The desired motor speed.
 * @property [maxMotorTorque] - The maximum torque the motor can apply.
 * @property [enableSpring] - Whether to enable the joint's spring.
 * @property [hertz] - The frequency of the joint's spring.
 * @property [dampingRatio] - The damping ratio of the joint's spring.
 * @property [collideConnected] - Whether the connected bodies should collide.
 * @property [drawSize] - The size to use when drawing the joint.
 */
declare type RevoluteJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2RevoluteJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    anchorA?: b2Vec2;
    anchorB?: b2Vec2;
    lowerAngle?: number;
    upperAngle?: number;
    enableLimit?: boolean;
    enableMotor?: boolean;
    motorSpeed?: number;
    maxMotorTorque?: number;
    enableSpring?: boolean;
    hertz?: number;
    dampingRatio?: number;
    collideConnected?: boolean;
    drawSize?: number;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2WeldJointDef.
 * @property bodyIdA - The first body to weld with this joint.
 * @property bodyIdB - The second body to weld with this joint.
 * @property [anchorA] - Local position of the anchor point on the first body.
 * @property [anchorB] - Local position of the anchor point on the second body.
 * @property [hertz] - The frequency at which the weld joint is enforced.
 * @property [dampingRatio] - The angular damping ratio when the weld joint is springing back into alignment.
 * @property [referenceAngle] - Reference angle for the weld joint at rest.
 * @property [collideConnected] - Whether the connected bodies should collide.
 */
declare type WeldJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2WeldJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    anchorA?: b2Vec2;
    anchorB?: b2Vec2;
    hertz?: number;
    dampingRatio?: number;
    referenceAngle?: number;
    collideConnected?: boolean;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2DistanceJointDef.
 * @property bodyIdA - The first body to connect with this joint.
 * @property bodyIdB - The second body to connect with this joint.
 * @property [anchorA] - Local position of the anchor point on the first body.
 * @property [anchorB] - Local position of the anchor point on the second body.
 * @property [length] - The natural length of the joint.
 * @property [minLength] - The minimum allowed length of the joint.
 * @property [maxLength] - The maximum allowed length of the joint.
 * @property [enableSpring] - Whether to enable the joint's spring.
 * @property [hertz] - The frequency of the joint's spring.
 * @property [dampingRatio] - The damping ratio of the joint's spring.
 * @property [enableLimit] - Whether to enable length limits.
 * @property [collideConnected] - Whether the connected bodies should collide.
 */
declare type DistanceJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2DistanceJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    anchorA?: b2Vec2;
    anchorB?: b2Vec2;
    length?: number;
    minLength?: number;
    maxLength?: number;
    enableSpring?: boolean;
    hertz?: number;
    dampingRatio?: number;
    enableLimit?: boolean;
    collideConnected?: boolean;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2WheelJointDef.
 * @property bodyIdA - The first body to connect with this joint.
 * @property bodyIdB - The second body to connect with this joint.
 * @property [anchorA] - Local position of the anchor point on the first body.
 * @property [anchorB] - Local position of the anchor point on the second body.
 * @property [enableSpring] - Whether to enable the joint's spring.
 * @property [axis] - The local axis for the joint movement on body A.
 * @property [hertz] - The frequency of the joint's spring.
 * @property [dampingRatio] - The damping ratio of the joint's spring.
 * @property [enableLimit] - Whether to enable translation limits.
 * @property [lowerTranslation] - The lower translation limit.
 * @property [upperTranslation] - The upper translation limit.
 * @property [enableMotor] - Whether to enable the joint's motor.
 * @property [maxMotorTorque] - The maximum torque the motor can apply.
 * @property [motorSpeed] - The desired motor speed.
 * @property [collideConnected] - Whether the connected bodies should collide.
 */
declare type WheelJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2WheelJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    anchorA?: b2Vec2;
    anchorB?: b2Vec2;
    enableSpring?: boolean;
    axis?: b2Vec2;
    hertz?: number;
    dampingRatio?: number;
    enableLimit?: boolean;
    lowerTranslation?: number;
    upperTranslation?: number;
    enableMotor?: boolean;
    maxMotorTorque?: number;
    motorSpeed?: number;
    collideConnected?: boolean;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2PrismaticJointDef.
 * @property bodyIdA - The first body to connect with this joint.
 * @property bodyIdB - The second body to connect with this joint.
 * @property [anchorA] - Local position of the anchor point on the first body.
 * @property [anchorB] - Local position of the anchor point on the second body.
 * @property [axis] - The local axis for the joint movement on body A.
 * @property [referenceAngle] - The reference angle between the bodies.
 * @property [enableSpring] - Whether to enable the joint's spring.
 * @property [hertz] - The frequency of the joint's spring.
 * @property [dampingRatio] - The damping ratio of the joint's spring.
 * @property [enableLimit] - Whether to enable translation limits.
 * @property [lowerTranslation] - The lower translation limit.
 * @property [upperTranslation] - The upper translation limit.
 * @property [enableMotor] - Whether to enable the joint's motor.
 * @property [maxMotorForce] - The maximum force the motor can apply.
 * @property [motorSpeed] - The desired motor speed.
 * @property [collideConnected] - Whether the connected bodies should collide.
 */
declare type PrismaticJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2PrismaticJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    anchorA?: b2Vec2;
    anchorB?: b2Vec2;
    axis?: b2Vec2;
    referenceAngle?: number;
    enableSpring?: boolean;
    hertz?: number;
    dampingRatio?: number;
    enableLimit?: boolean;
    lowerTranslation?: number;
    upperTranslation?: number;
    enableMotor?: boolean;
    maxMotorForce?: number;
    motorSpeed?: number;
    collideConnected?: boolean;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2MotorJointDef.
 * @property bodyIdA - The first body to connect with this joint.
 * @property bodyIdB - The second body to connect with this joint.
 * @property [linearOffset] - The desired linear offset in frame A.
 * @property [maxForce] - The maximum force that can be applied to reach the target offsets.
 * @property [angularOffset] - The desired angular offset.
 * @property [maxTorque] - The maximum torque that can be applied to reach the target angular offset.
 * @property [correctionFactor] - Position correction factor in the range [0,1].
 * @property [collideConnected] - Whether the connected bodies should collide.
 */
declare type MotorJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2MotorJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    linearOffset?: b2Vec2;
    maxForce?: number;
    angularOffset?: number;
    maxTorque?: number;
    correctionFactor?: number;
    collideConnected?: boolean;
};

/**
 * @property worldId - ID for the world in which the bodies and joint exist.
 * @property [jointDef] - A pre-existing b2MouseJointDef.
 * @property bodyIdA - The first (usually static) body to connect with this joint.
 * @property bodyIdB - The second (usually dynamic) body to connect with this joint.
 * @property [target] - The initial world target point.
 * @property [hertz] - The response frequency.
 * @property [dampingRatio] - The damping ratio.
 * @property [maxForce] - The maximum force that can be exerted to reach the target point.
 * @property [collideConnected] - Whether the connected bodies should collide.
 * e.g. worldId:worldId, bodyIdA:mouseCircle.bodyId, bodyIdB:mouseBox.bodyId, target:new b2Vec2(0, 0), hertz:30.0, dampingRatio:0.999, maxForce:35000
 */
declare type MouseJointConfig = {
    worldId: b2WorldId;
    jointDef?: b2MouseJointDef;
    bodyIdA: b2BodyId;
    bodyIdB: b2BodyId;
    target?: b2Vec2;
    hertz?: number;
    dampingRatio?: number;
    maxForce?: number;
    collideConnected?: boolean;
};

declare namespace PrismaticJoint { }

/**
 * Sets the spring state of a prismatic joint. When the spring state changes,
 * the spring impulse is reset to zero. If the state doesn't change, no action is taken.
 * @param jointId - The identifier of the prismatic joint to modify.
 * @param enableSpring - Whether to enable (true) or disable (false) the spring.
 */
declare function b2PrismaticJoint_EnableSpring(jointId: b2JointId, enableSpring: boolean): void;

/**
 * @param jointId - The identifier for the prismatic joint to check.
 * @returns True if the spring mechanism is enabled, false otherwise.
 */
declare function b2PrismaticJoint_IsSpringEnabled(jointId: b2JointId): boolean;

/**
 * Updates the spring frequency of a prismatic joint. The joint must be of type b2_prismaticJoint.
 * @param jointId - The identifier for the prismatic joint to modify.
 * @param hertz - The spring frequency in Hertz.
 */
declare function b2PrismaticJoint_SetSpringHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the spring frequency in Hertz for a prismatic joint.
 * @param jointId - The identifier for the prismatic joint.
 * @returns The spring frequency in Hertz.
 */
declare function b2PrismaticJoint_GetSpringHertz(jointId: b2JointId): number;

/**
 * Sets the spring damping ratio for a prismatic joint. The joint must be of type b2_prismaticJoint.
 * @param jointId - The identifier for the prismatic joint.
 * @param dampingRatio - The damping ratio for the spring.
 */
declare function b2PrismaticJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the spring damping ratio of a prismatic joint.
 * @param jointId - The identifier for the prismatic joint.
 * @returns The spring damping ratio value.
 */
declare function b2PrismaticJoint_GetSpringDampingRatio(jointId: b2JointId): number;

/**
 * Enables or disables the translation limits on a prismatic joint. When the limit is disabled,
 * the joint's limit impulses are reset to zero.
 * @param jointId - The identifier for the prismatic joint
 * @param enableLimit - True to enable the translation limit, false to disable it
 */
declare function b2PrismaticJoint_EnableLimit(jointId: b2JointId, enableLimit: boolean): void;

/**
 * @param jointId - The identifier for the prismatic joint to check.
 * @returns True if the limit is enabled for the joint, false otherwise.
 */
declare function b2PrismaticJoint_IsLimitEnabled(jointId: b2JointId): boolean;

/**
 * @param jointId - The identifier for the prismatic joint.
 * @returns The lower translation limit of the prismatic joint.
 */
declare function b2PrismaticJoint_GetLowerLimit(jointId: b2JointId): number;

/**
 * @param jointId - The identifier for the prismatic joint.
 * @returns The upper translation limit of the prismatic joint.
 */
declare function b2PrismaticJoint_GetUpperLimit(jointId: b2JointId): number;

/**
 * Sets new translation limits for a prismatic joint. The function automatically
 * orders the limits so that the lower value is always less than or equal to
 * the upper value. When the limits change, the joint's impulses are reset to zero.
 * @param jointId - The identifier for the prismatic joint.
 * @param lower - The lower translation limit.
 * @param upper - The upper translation limit.
 */
declare function b2PrismaticJoint_SetLimits(jointId: b2JointId, lower: number, upper: number): void;

/**
 * Enables or disables the motor on a prismatic joint. When the motor is disabled,
 * the motor impulse is reset to zero.
 * @param jointId - The identifier for the prismatic joint
 * @param enableMotor - True to enable the motor, false to disable it
 */
declare function b2PrismaticJoint_EnableMotor(jointId: b2JointId, enableMotor: boolean): void;

/**
 * @param jointId - The identifier for the prismatic joint to check.
 * @returns True if the motor is enabled, false otherwise.
 */
declare function b2PrismaticJoint_IsMotorEnabled(jointId: b2JointId): boolean;

/**
 * @param jointId - The identifier for the prismatic joint to modify.
 * @param motorSpeed - The desired motor speed in radians per second.
 */
declare function b2PrismaticJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: number): void;

/**
 * Gets the motor speed of a prismatic joint.
 * @param jointId - The identifier for the prismatic joint.
 * @returns The current motor speed of the prismatic joint.
 */
declare function b2PrismaticJoint_GetMotorSpeed(jointId: b2JointId): number;

/**
 * Gets the current motor force of a prismatic joint. The force is calculated by
 * multiplying the joint's motor impulse by the inverse of the world's time step.
 * @param jointId - The ID of the prismatic joint
 * @returns The current motor force in Newtons
 */
declare function b2PrismaticJoint_GetMotorForce(jointId: b2JointId): number;

/**
 * Sets the maximum motor force for a prismatic joint.
 * @param jointId - The identifier for the prismatic joint.
 * @param force - The maximum force the motor can apply.
 */
declare function b2PrismaticJoint_SetMaxMotorForce(jointId: b2JointId, force: number): void;

/**
 * Gets the maximum motor force of a prismatic joint.
 * @param jointId - The identifier for the prismatic joint.
 * @returns The maximum force that can be applied by the joint's motor.
 */
declare function b2PrismaticJoint_GetMaxMotorForce(jointId: b2JointId): number;

declare namespace Ragdoll { }

declare namespace RevoluteJoint { }

/**
 * Enables or disables the spring functionality of a revolute joint.
 * When the spring state changes, the spring impulse is reset to zero.
 * @param jointId - The identifier of the revolute joint to modify
 * @param enableSpring - True to enable the spring, false to disable it
 */
declare function b2RevoluteJoint_EnableSpring(jointId: b2JointId, enableSpring: boolean): void;

/**
 * @param jointId - The identifier for the revolute joint to check.
 * @returns True if spring functionality is enabled, false otherwise.
 */
declare function b2RevoluteJoint_IsSpringEnabled(jointId: b2JointId): boolean;

/**
 * Sets the spring oscillation frequency for a revolute joint. The joint must be
 * of type b2_revoluteJoint.
 * @param jointId - The identifier for the revolute joint to modify.
 * @param hertz - The spring frequency in Hertz (Hz).
 */
declare function b2RevoluteJoint_SetSpringHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the spring frequency in Hertz for a revolute joint.
 * @param jointId - The identifier for the revolute joint.
 * @returns The spring frequency in Hertz.
 */
declare function b2RevoluteJoint_GetSpringHertz(jointId: b2JointId): number;

/**
 * Sets the damping ratio for a revolute joint's spring.
 * @param jointId - The identifier for the revolute joint.
 * @param dampingRatio - The damping ratio for the spring.
 */
declare function b2RevoluteJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the spring damping ratio of a revolute joint.
 * @param jointId - The identifier of the revolute joint.
 * @returns The spring damping ratio value of the revolute joint.
 */
declare function b2RevoluteJoint_GetSpringDampingRatio(jointId: b2JointId): number;

/**
 * Calculates the relative angle between two bodies connected by a revolute joint by
 * comparing their transforms and subtracting the joint's reference angle. The result
 * is unwound to ensure consistent angle representation.
 * @param jointId - The identifier for the revolute joint.
 * @returns The current angle in radians between the two connected bodies,
 * relative to the reference angle.
 */
declare function b2RevoluteJoint_GetAngle(jointId: b2JointId): number;

/**
 * When enabled, the joint will restrict rotation to be between its upper and lower angle limits.
 * When the limit state changes, the joint's limit impulses are reset to zero.
 * @param jointId - The identifier for the revolute joint.
 * @param enableLimit - True to enable the joint limits, false to disable them.
 */
declare function b2RevoluteJoint_EnableLimit(jointId: b2JointId, enableLimit: boolean): void;

/**
 * @param jointId - The identifier for the revolute joint to check.
 * @returns True if the joint's limit is enabled, false otherwise.
 */
declare function b2RevoluteJoint_IsLimitEnabled(jointId: b2JointId): boolean;

/**
 * Gets the lower angle limit of a revolute joint.
 * @param jointId - The identifier for the revolute joint.
 * @returns The lower angle limit in radians.
 */
declare function b2RevoluteJoint_GetLowerLimit(jointId: b2JointId): number;

/**
 * Gets the upper angle limit of a revolute joint.
 * @param jointId - The identifier for the revolute joint.
 * @returns The upper angle limit in radians.
 */
declare function b2RevoluteJoint_GetUpperLimit(jointId: b2JointId): number;

/**
 * Sets the lower and upper angle limits for a revolute joint. The function automatically
 * orders the limits so that the lower value is always less than or equal to the upper value.
 * When the limits change, the joint impulses are reset to zero.
 * @param jointId - The identifier for the revolute joint to modify
 * @param lower - The lower angle limit in radians
 * @param upper - The upper angle limit in radians
 */
declare function b2RevoluteJoint_SetLimits(jointId: b2JointId, lower: number, upper: number): void;

/**
 * Enables or disables the motor on a revolute joint. When the motor is disabled,
 * its accumulated impulse is reset to zero.
 * @param jointId - The identifier for the revolute joint
 * @param enableMotor - True to enable the joint's motor, false to disable it
 */
declare function b2RevoluteJoint_EnableMotor(jointId: b2JointId, enableMotor: boolean): void;

/**
 * @param jointId - The identifier for the revolute joint to check.
 * @returns True if the motor is enabled, false otherwise.
 */
declare function b2RevoluteJoint_IsMotorEnabled(jointId: b2JointId): boolean;

/**
 * Sets the angular velocity for the motor of a revolute joint. A positive velocity
 * means the joint will rotate counterclockwise.
 * @param jointId - The identifier for the revolute joint.
 * @param motorSpeed - The desired motor speed in radians per second.
 */
declare function b2RevoluteJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: number): void;

/**
 * Gets the motor speed of a revolute joint.
 * @param jointId - The identifier of the revolute joint.
 * @returns The current motor speed in radians per second.
 */
declare function b2RevoluteJoint_GetMotorSpeed(jointId: b2JointId): number;

/**
 * Calculates the motor torque by multiplying the motor impulse by the inverse
 * of the time step. The joint must be of type b2_revoluteJoint.
 * @param jointId - The identifier for the revolute joint.
 * @returns The current motor torque in Newton-meters.
 */
declare function b2RevoluteJoint_GetMotorTorque(jointId: b2JointId): number;

/**
 * Sets the maximum motor torque for a revolute joint.
 * @param jointId - The identifier for the revolute joint.
 * @param torque - The maximum motor torque value to set.
 */
declare function b2RevoluteJoint_SetMaxMotorTorque(jointId: b2JointId, torque: number): void;

/**
 * Gets the maximum motor torque of a revolute joint.
 * @param jointId - The identifier for the revolute joint.
 * @returns The maximum motor torque value.
 */
declare function b2RevoluteJoint_GetMaxMotorTorque(jointId: b2JointId): number;

declare namespace Shape { }

/**
 * Creates a circle shape and attaches it to a body.
 * @param bodyId - The identifier of the body to attach the shape to
 * @param def - The shape definition containing properties like friction and density
 * @param circle - The circle geometry definition
 * @returns The identifier of the created circle shape
 */
declare function b2CreateCircleShape(bodyId: b2BodyId, def: b2ShapeDef, circle: b2Circle): b2ShapeId;

/**
 * Creates a capsule shape if the distance between centers is greater than b2_linearSlop.
 * If centers are closer than b2_linearSlop, creates a circle shape instead with radius
 * equal to the capsule radius and center at the midpoint between capsule centers.
 * @param bodyId - The body ID to attach the shape to
 * @param def - The shape definition parameters
 * @param capsule - The capsule geometry containing center1, center2 and radius
 * @returns The ID of the created shape
 */
declare function b2CreateCapsuleShape(bodyId: b2BodyId, def: b2ShapeDef, capsule: b2Capsule): b2ShapeId;

/**
 * Creates a polygon shape and attaches it to a body.
 * @param bodyId - The identifier of the body to attach the shape to
 * @param def - The shape definition containing common shape properties
 * @param polygon - The polygon data including vertices and radius
 * @returns The identifier of the created polygon shape
 */
declare function b2CreatePolygonShape(bodyId: b2BodyId, def: b2ShapeDef, polygon: b2Polygon): b2ShapeId;

/**
 * Creates a segment shape between two points and attaches it to the specified body.
 * The function validates that the segment length is greater than b2_linearSlop
 * before creating the shape.
 * @param bodyId - The ID of the body to attach the shape to
 * @param def - The shape definition parameters
 * @param segment - The segment geometry defined by point1 and point2
 * @returns The ID of the created segment shape
 */
declare function b2CreateSegmentShape(bodyId: b2BodyId, def: b2ShapeDef, segment: b2Segment): b2ShapeId;

/**
 * Removes a shape from the physics world. If the parent body has automatic mass calculation enabled,
 * the body's mass properties are recalculated after the shape is destroyed.
 * The function performs the following operations:
 * 1. Retrieves the world and shape from the provided shapeId
 * 2. Destroys the shape internally
 * 3. Updates the parent body's mass data if automatic mass calculation is enabled
 * @param shapeId - The identifier of the shape to destroy.
 */
declare function b2DestroyShape(shapeId: b2ShapeId): void;

/**
 * Creates a chain shape composed of connected line segments attached to a body.
 * The chain can be either a closed loop or an open chain.
 * @param bodyId - The ID of the body to attach the chain to
 * @param def - The chain definition containing:
 * - {number} count - Number of vertices (must be >= 4)
 * - {b2Vec2[]} points - Array of vertex points
 * - {boolean} isLoop - Whether the chain forms a closed loop
 * - {number} friction - Friction coefficient (must be >= 0)
 * - {number} restitution - Restitution coefficient (must be >= 0)
 * - {b2Filter} filter - Collision filter data
 * - {*} userData - User data pointer
 * @returns The ID of the created chain shape
 */
declare function b2CreateChain(bodyId: b2BodyId, def: b2ChainDef): b2ChainId;

/**
 * Destroys a chain of shapes attached to a body in a Box2D world. The function removes all shapes
 * associated with the chain and frees the chain ID from the world's chain ID pool.
 * @param chainId - The identifier for the chain to be destroyed, containing world and index information
 */
declare function b2DestroyChain(chainId: b2ChainId): void;

/**
 * Retrieves the body ID for a given shape by first accessing the world object,
 * then getting the shape from the world, and finally creating a body ID
 * from the shape's stored body reference.
 * @param shapeId - The ID of the shape to query.
 * @returns The ID of the body that owns the shape.
 */
declare function b2Shape_GetBody(shapeId: b2ShapeId): b2BodyId;

/**
 * Associates arbitrary user data with a shape identified by shapeId. The shape must exist
 * in the world referenced by the shapeId. The function retrieves the shape from the world
 * and updates its userData property.
 * @param shapeId - The identifier of the shape to modify.
 * @param userData - The user data to associate with the shape.
 */
declare function b2Shape_SetUserData(shapeId: b2ShapeId, userData: any): void;

/**
 * This function retrieves the user data that was previously attached to a shape
 * by looking up the shape in the world using the provided shape ID.
 * @param shapeId - The identifier for the shape.
 * @returns The user data associated with the shape.
 */
declare function b2Shape_GetUserData(shapeId: b2ShapeId): void;

/**
 * Retrieves a shape from the physics world using its ID and returns
 * whether it is configured as a sensor. Sensor shapes detect collisions
 * but do not generate physical responses.
 * @param shapeId - The identifier for the shape to check.
 * @returns True if the shape is a sensor, false otherwise.
 */
declare function b2Shape_IsSensor(shapeId: b2ShapeId): boolean;

/**
 * Transforms the test point into the shape's local space and performs
 * point-in-shape testing based on the shape type (capsule, circle, or polygon).
 * Returns false for unrecognized shape types.
 * @param shapeId - The identifier for the shape to test against
 * @param point - The world space point to test
 * @returns True if the point is inside the shape, false otherwise
 */
declare function b2Shape_TestPoint(shapeId: b2ShapeId, point: b2Vec2): boolean;

/**
 * Performs a ray cast against a shape in world space, transforming the input/output
 * between local and world coordinates.
 * @param shapeId - The identifier for the shape to test
 * @param origin - The starting point of the ray in world coordinates
 * @param translation - The direction and length of the ray in world coordinates
 * @returns The ray cast results containing:
 * - hit: boolean indicating if the ray intersects the shape
 * - point: intersection point in world coordinates (if hit is true)
 * - normal: surface normal at intersection in world coordinates (if hit is true)
 * - fraction: fraction of translation where intersection occurs (if hit is true)
 */
declare function b2Shape_RayCast(shapeId: b2ShapeId, origin: b2Vec2, translation: b2Vec2): b2CastOutput;

/**
 * Sets a new density value for the specified shape. If the new density matches
 * the current density, no changes are made. The function performs validation
 * to ensure the density is a valid non-negative number.
 * @param shapeId - The identifier of the shape to modify.
 * @param density - The new density value. Must be non-negative.
 */
declare function b2Shape_SetDensity(shapeId: b2ShapeId, density: number): void;

/**
 * Retrieves the density property of a shape by first accessing the world object
 * using the world identifier stored in the shapeId, then accessing the specific
 * shape within that world.
 * @param shapeId - The identifier for the shape within a Box2D world.
 * @returns The density value of the specified shape.
 */
declare function b2Shape_GetDensity(shapeId: b2ShapeId): number;

/**
 * Sets a new friction value for the specified shape. The operation will not proceed
 * if the physics world is locked. The friction parameter must be a valid number
 * greater than or equal to zero.
 * @param shapeId - The identifier for the shape to modify
 * @param friction - The friction coefficient value (must be >= 0)
 */
declare function b2Shape_SetFriction(shapeId: b2ShapeId, friction: number): void;

/**
 * Retrieves the friction value from a shape object in the Box2D physics world
 * using the provided shape identifier.
 * @param shapeId - The identifier for the shape in the physics world.
 * @returns The friction coefficient of the shape.
 */
declare function b2Shape_GetFriction(shapeId: b2ShapeId): number;

/**
 * Sets the restitution (bounciness) value for a shape.
 * @param shapeId - The identifier for the shape to modify.
 * @param restitution - The restitution value to set. Must be non-negative.
 */
declare function b2Shape_SetRestitution(shapeId: b2ShapeId, restitution: number): void;

/**
 * Retrieves the restitution (bounciness) value associated with the specified shape
 * from the physics world.
 * @param shapeId - The identifier for the shape in the physics world.
 * @returns The restitution coefficient of the shape.
 */
declare function b2Shape_GetRestitution(shapeId: b2ShapeId): number;

/**
 * Retrieves the collision filtering data from a shape by first accessing the world
 * object using the world identifier stored in the shapeId, then accessing the
 * shape within that world using the shapeId.
 * @param shapeId - The identifier for the shape.
 * @returns The collision filter data associated with the shape.
 */
declare function b2Shape_GetFilter(shapeId: b2ShapeId): b2Filter;

/**
 * Updates the collision filter properties of a shape. If the new filter settings
 * match the existing ones, no changes are made. When the categoryBits change,
 * the shape's broad-phase proxy is destroyed and recreated. The function also
 * wakes connected bodies when the filter changes.
 * @param shapeId - The identifier for the shape to modify.
 * @param filter - The new collision filter settings to apply.
 */
declare function b2Shape_SetFilter(shapeId: b2ShapeId, filter: b2Filter): void;

/**
 * Sets the enableSensorEvents property of a shape in the physics world. The shape
 * must exist in a valid world context for the operation to succeed.
 * @param shapeId - The identifier of the shape to modify.
 * @param flag - True to enable sensor events, false to disable them.
 */
declare function b2Shape_EnableSensorEvents(shapeId: b2ShapeId, flag: boolean): void;

/**
 * This function retrieves a shape from the physics world using its ID and returns
 * the state of its sensor events flag.
 * @param shapeId - The identifier for the shape to check.
 * @returns True if sensor events are enabled for the shape, false otherwise.
 */
declare function b2Shape_AreSensorEventsEnabled(shapeId: b2ShapeId): boolean;

/**
 * Sets whether a shape should generate contact events during collision detection.
 * The shape must belong to a valid world, otherwise the function returns without effect.
 * @param shapeId - The identifier for the shape.
 * @param flag - True to enable contact events, false to disable.
 */
declare function b2Shape_EnableContactEvents(shapeId: b2ShapeId, flag: boolean): void;

/**
 * This function retrieves a shape from the physics world using its ID and checks
 * the enableContactEvents property of that shape.
 * @param shapeId - The identifier for the shape to check.
 * @returns True if contact events are enabled for the shape, false otherwise.
 */
declare function b2Shape_AreContactEventsEnabled(shapeId: b2ShapeId): boolean;

/**
 * Enables or disables pre-solve events for a specific shape in the physics simulation.
 * The function first validates the world reference and returns if invalid.
 * If valid, it updates the shape's pre-solve events setting according to the flag parameter.
 * @param shapeId - The identifier for the shape in the physics world
 * @param flag - Boolean value to enable or disable pre-solve events for the shape
 */
declare function b2Shape_EnablePreSolveEvents(shapeId: b2ShapeId, flag: boolean): void;

/**
 * This function retrieves a shape from the physics world using its ID and checks
 * the enablePreSolveEvents property of that shape.
 * @param shapeId - The identifier for the shape to check.
 * @returns True if pre-solve events are enabled for the shape, false otherwise.
 */
declare function b2Shape_ArePreSolveEventsEnabled(shapeId: b2ShapeId): boolean;

/**
 * Sets whether a shape should generate hit events during collision detection.
 * The shape must belong to a valid world, otherwise the function returns without effect.
 * @param shapeId - The identifier of the shape to modify.
 * @param flag - True to enable hit events, false to disable.
 */
declare function b2Shape_EnableHitEvents(shapeId: b2ShapeId, flag: boolean): void;

/**
 * This function retrieves a shape from the physics world using its ID and checks
 * if hit events are enabled for that shape by accessing the enableHitEvents property.
 * @param shapeId - The identifier for the shape to check.
 * @returns True if hit events are enabled for the shape, false otherwise.
 */
declare function b2Shape_AreHitEventsEnabled(shapeId: b2ShapeId): boolean;

/**
 * Retrieves a shape from the physics world using the provided shape ID
 * and returns its type classification.
 * @param shapeId - The ID of the shape to query
 * @returns The type of the specified shape
 */
declare function b2Shape_GetType(shapeId: b2ShapeId): b2ShapeType;

/**
 * Retrieves a circle shape from the physics world using the provided shape identifier.
 * @param shapeId - A shape identifier containing world and shape reference
 * @returns The circle shape object
 */
declare function b2Shape_GetCircle(shapeId: b2ShapeId): b2Circle;

/**
 * Retrieves the segment data from a shape in the physics world. The shape must be of type b2_segmentShape.
 * @param shapeId - A shape identifier containing world and shape reference
 * @returns The segment data from the specified shape
 */
declare function b2Shape_GetSegment(shapeId: b2ShapeId): b2Segment;

/**
 * Gets the chain segment data from a shape identified by its ID.
 * @param shapeId - An object containing the shape identifier and world reference.
 * @param shapeId.world0 - The world identifier.
 * @returns The chain segment data associated with the shape.
 */
declare function b2Shape_GetChainSegment(shapeId: {
    world0: number;
}): any;

/**
 * Retrieves the capsule shape data from a shape in the physics world using the provided shape ID.
 * @param shapeId - A shape identifier containing world and shape reference information
 * @returns The capsule shape data associated with the given shape ID
 */
declare function b2Shape_GetCapsule(shapeId: b2ShapeId): b2Capsule;

/**
 * Gets a polygon shape from a shape ID.
 * @param shapeId - The ID of the shape to retrieve.
 * @returns The polygon shape associated with the given shape ID.
 */
declare function b2Shape_GetPolygon(shapeId: b2ShapeId): b2Polygon;

/**
 * Sets a shape's type to circle and updates its properties. The function updates
 * the shape's proxy in the broad-phase collision system and can wake connected bodies.
 * If the world is locked or invalid, the function returns without making changes.
 * @param shapeId - The identifier for the shape to be modified
 * @param circle - The circle shape configuration to set
 */
declare function b2Shape_SetCircle(shapeId: b2ShapeId, circle: b2Circle): void;

/**
 * Sets a shape's type to capsule and updates its configuration. The function
 * resets the shape's proxy in the broad-phase collision system, optionally
 * waking connected bodies and destroying the existing proxy.
 * @param shapeId - The identifier for the shape to be modified
 * @param capsule - The capsule shape configuration to set
 */
declare function b2Shape_SetCapsule(shapeId: b2ShapeId, capsule: b2Capsule): void;

/**
 * Sets a shape's type to segment and updates its segment data. After updating,
 * it resets the shape's collision proxy in the physics world. The function requires
 * a valid world lock to execute successfully.
 * @param shapeId - The identifier for the shape to be modified
 * @param segment - The segment data to assign to the shape
 */
declare function b2Shape_SetSegment(shapeId: b2ShapeId, segment: b2Segment): void;

/**
 * Sets a shape's type to polygon and assigns polygon data to it. The function
 * updates the shape's proxy in the broad-phase collision system and can wake
 * connected bodies.
 * @param shapeId - The identifier for the shape to be modified
 * @param polygon - The polygon data to assign to the shape
 */
declare function b2Shape_SetPolygon(shapeId: b2ShapeId, polygon: b2Polygon): void;

/**
 * Retrieves the parent chain identifier for a given shape if it is a chain segment shape
 * and has an associated chain. The function checks if the shape is of type b2_chainSegmentShape
 * and has a valid chain reference before returning the chain identifier.
 * @param shapeId - The identifier of the shape to check for parent chain
 * @returns A b2ChainId object. Returns an empty b2ChainId (default values) if the shape
 * is not a chain segment shape or has no parent chain
 */
declare function b2Shape_GetParentChain(shapeId: b2ShapeId): b2ChainId;

/**
 * Updates the friction property of each shape within the specified chain. The function
 * retrieves the chain shape from the world, then iterates through all shapes in the
 * chain and sets their friction to the specified value.
 * @param chainId - The identifier for the chain whose friction will be modified
 * @param friction - The friction value to set for all shapes in the chain
 */
declare function b2Chain_SetFriction(chainId: b2ChainId, friction: number): void;

/**
 * Sets the restitution coefficient for all shapes that make up the specified chain.
 * If the world is not found using the provided chainId, the function returns without making changes.
 * @param chainId - The identifier for the chain whose restitution will be set
 * @param restitution - The restitution value to apply to all shapes in the chain
 */
declare function b2Chain_SetRestitution(chainId: b2ChainId, restitution: number): void;

/**
 * Retrieves the number of contacts associated with the body that owns the specified shape.
 * If the shape is a sensor or the world reference is invalid, the function returns 0.
 * @param shapeId - The identifier for the shape to check
 * @returns The number of contacts for the shape's body. Returns 0 if the world is invalid,
 * or if the shape is a sensor.
 */
declare function b2Shape_GetContactCapacity(shapeId: b2ShapeId): number;

/**
 * Retrieves contact data for a specified shape. For each valid contact involving the shape,
 * stores the shape IDs of both bodies in contact and their contact manifold.
 * Only stores contacts where the touching flag is set and ignores sensor shapes.
 * @param shapeId - The identifier of the shape to get contact data for
 * @param contactData - Array to store the contact data
 * @param capacity - Maximum number of contacts to retrieve
 * @returns The number of contacts found and stored in contactData
 */
declare function b2Shape_GetContactData(shapeId: b2ShapeId, contactData: { shapeIdA: b2ShapeId; shapeIdB: b2ShapeId; manifold: b2Manifold; }[], capacity: number): number;

/**
 * Retrieves the Axis-Aligned Bounding Box (AABB) associated with a shape in the physics world.
 * If the world reference is invalid, returns a default AABB with zero dimensions.
 * @param shapeId - The identifier for the shape.
 * @returns The AABB of the shape. Returns an empty AABB if the world is null.
 */
declare function b2Shape_GetAABB(shapeId: b2ShapeId): b2AABB;

/**
 * Calculates the closest point on a shape to a given target point, taking into account
 * the shape's position and rotation in world space. Uses the distance calculation
 * algorithm with shape proxies and transforms.
 * @param shapeId - ID of the shape to query
 * @param target - The target point to find the closest point to
 * @returns The closest point on the shape to the target point. Returns (0,0) if the world is invalid.
 */
declare function b2Shape_GetClosestPoint(shapeId: b2ShapeId, target: b2Vec2): b2Vec2;

declare namespace Solver { }

/**
 * function b2PrepareJointsTask(startIndex, endIndex, context)
 * {
 *     const joints = context.joints;
 *
 *     for (let i = startIndex; i < endIndex; ++i) {
 *         const joint = joints[i];
 *         joint_h.b2PrepareJoint(joint, context);
 *     }
 * }
 */
declare function b2IntegratePositionsTask(): void;

declare namespace SolverSet { }

declare namespace StackAllocator { }

declare namespace Table { }

declare namespace Types { }

/**
 * Initializes a new b2WorldDef with default values suitable for standard physics simulations.
 * All distance-based values are scaled by b2_lengthUnitsPerMeter2.
 * @returns A world definition object with the following properties:
 * - gravity: {b2Vec2} Set to (0, -10)
 * - hitEventThreshold: {number} Set to 1 meter
 * - restitutionThreshold: {number} Set to 10 meters
 * - contactPushoutVelocity: {number} Set to 5 meters
 * - contactHertz: {number} Set to 30
 * - contactDampingRatio: {number} Set to 10
 * - jointHertz: {number} Set to 60
 * - jointDampingRatio: {number} Set to 5
 * - maximumLinearVelocity: {number} Set to 400 meters
 * - enableSleep: {boolean} Set to true
 * - enableContinuous: {boolean} Set to true
 */
declare function b2DefaultWorldDef(): b2WorldDef;

/**
 * Creates a new b2BodyDef with default values.
 * @returns A body definition object with the following default properties:
 * - type: b2_staticBody
 * - position: (0,0)
 * - rotation: (cos=1, sin=0)
 * - linearVelocity: (0,0)
 * - angularVelocity: 0
 * - linearDamping: 0
 * - angularDamping: 0
 * - gravityScale: 1
 * - sleepThreshold: 0.05 * b2_lengthUnitsPerMeter2
 * - userData: null
 * - enableSleep: true
 * - isAwake: true
 * - fixedRotation: false
 * - isBullet: false
 * - isEnabled: true
 * - updateBodyMass: true
 * - allowFastRotation: false
 */
declare function b2DefaultBodyDef(): b2BodyDef;

/**
 * Creates and returns a new b2Filter object initialized with default values for collision filtering.
 * The categoryBits determine what collision category this object belongs to,
 * the maskBits determine what categories this object can collide with,
 * and the groupIndex determines collision groups (negative values mean never collide,
 * positive values mean always collide with same group).
 * @returns A filter object with categoryBits=1, maskBits=4294967295 (0xFFFFFFFF), and groupIndex=0
 */
declare function b2DefaultFilter(): b2Filter;

/**
 * This function instantiates a new b2QueryFilter with default collision filtering
 * settings. The categoryBits value of 1 represents the default category, while
 * the maskBits value of 4294967295 allows collision with all categories.
 * @returns A query filter object with categoryBits set to 1 and
 * maskBits set to 4294967295 (0xFFFFFFFF).
 */
declare function b2DefaultQueryFilter(): b2QueryFilter;

/**
 * This function initializes a new b2ShapeDef object with commonly used physics
 * properties. The shape definition can be used to create various types of
 * physics shapes in Box2D.
 * @returns A shape definition object with the following default values:
 * - friction: 0.6
 * - density: 1
 * - restitution: 0.1
 * - filter: default collision filter
 * - enableSensorEvents: true
 * - enableContactEvents: true
 */
declare function b2DefaultShapeDef(): b2ShapeDef;

/**
 * Initializes a new chain definition with common default values.
 * The chain shape represents a series of connected line segments.
 * @returns A chain definition object with:
 * - friction set to 0.6
 * - default filter settings
 * - all other properties at their default values
 */
declare function b2DefaultChainDef(): b2ChainDef;

declare namespace WeldJoint { }

/**
 * Sets the linear frequency (hertz) for a weld joint.
 * @param jointId - The identifier for the weld joint to modify
 * @param hertz - The frequency in hertz (must be >= 0)
 */
declare function b2WeldJoint_SetLinearHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the linear Hertz value from a weld joint.
 * @param jointId - The identifier for the weld joint.
 * @returns The linear Hertz value of the weld joint.
 */
declare function b2WeldJoint_GetLinearHertz(jointId: b2JointId): number;

/**
 * Sets the linear damping ratio for a weld joint.
 * @param jointId - The identifier for the weld joint to modify.
 * @param dampingRatio - The damping ratio value. Must be non-negative.
 */
declare function b2WeldJoint_SetLinearDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the linear damping ratio of a weld joint.
 * @param jointId - The identifier for the weld joint.
 * @returns The linear damping ratio of the weld joint.
 */
declare function b2WeldJoint_GetLinearDampingRatio(jointId: b2JointId): number;

/**
 * Sets the angular frequency (hertz) for a weld joint's angular spring-damper.
 * @param jointId - The identifier for the weld joint to modify.
 * @param hertz - The angular frequency in Hertz (must be >= 0).
 */
declare function b2WeldJoint_SetAngularHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the angular frequency (hertz) of a weld joint.
 * @param jointId - The identifier for the weld joint.
 * @returns The angular frequency in hertz.
 */
declare function b2WeldJoint_GetAngularHertz(jointId: b2JointId): number;

/**
 * Sets the angular damping ratio for a weld joint.
 * @param jointId - The identifier for the weld joint to modify.
 * @param dampingRatio - The angular damping ratio. Must be non-negative.
 */
declare function b2WeldJoint_SetAngularDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the angular damping ratio of a weld joint.
 * @param jointId - The identifier of the weld joint.
 * @returns The angular damping ratio of the weld joint.
 */
declare function b2WeldJoint_GetAngularDampingRatio(jointId: b2JointId): number;

declare namespace WheelJoint { }

/**
 * Enables or disables the spring functionality of a wheel joint. When the spring state
 * changes, the spring impulse is reset to zero.
 * @param jointId - The identifier for the wheel joint to modify
 * @param enableSpring - True to enable the spring, false to disable it
 */
declare function b2WheelJoint_EnableSpring(jointId: b2JointId, enableSpring: boolean): void;

/**
 * @param jointId - The identifier for the wheel joint to check.
 * @returns True if the spring is enabled, false otherwise.
 */
declare function b2WheelJoint_IsSpringEnabled(jointId: b2JointId): boolean;

/**
 * Sets the spring frequency for a wheel joint's oscillation. The frequency is specified
 * in Hertz (Hz). The joint must be of type b2_wheelJoint.
 * @param jointId - The identifier for the wheel joint to modify.
 * @param hertz - The spring frequency in Hertz (Hz).
 */
declare function b2WheelJoint_SetSpringHertz(jointId: b2JointId, hertz: number): void;

/**
 * Gets the spring frequency in Hertz for a wheel joint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The spring frequency in Hertz.
 */
declare function b2WheelJoint_GetSpringHertz(jointId: b2JointId): number;

/**
 * Sets the damping ratio for a wheel joint's spring.
 * @param jointId - The identifier for the wheel joint to modify
 * @param dampingRatio - The damping ratio for the spring (0 = no damping, 1 = critical damping)
 */
declare function b2WheelJoint_SetSpringDampingRatio(jointId: b2JointId, dampingRatio: number): void;

/**
 * Gets the damping ratio of a wheel joint's spring.
 * @param jointId - The identifier for the wheel joint.
 * @returns The spring damping ratio value.
 */
declare function b2WheelJoint_GetSpringDampingRatio(jointId: b2JointId): number;

/**
 * Sets whether the wheel joint's translation limit is active. When the limit is enabled,
 * the joint's translation will be constrained. When the limit state changes, the joint's
 * lower and upper impulses are reset to zero.
 * @param jointId - The identifier for the wheel joint.
 * @param enableLimit - True to enable the translation limit, false to disable it.
 */
declare function b2WheelJoint_EnableLimit(jointId: b2JointId, enableLimit: boolean): void;

/**
 * @param jointId - The identifier for the wheel joint to check.
 * @returns True if the limit is enabled, false otherwise.
 */
declare function b2WheelJoint_IsLimitEnabled(jointId: b2JointId): boolean;

/**
 * Gets the lower translation limit of a wheel joint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The lower translation limit of the wheel joint.
 */
declare function b2WheelJoint_GetLowerLimit(jointId: b2JointId): number;

/**
 * Gets the upper translation limit of a wheel joint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The upper translation limit of the wheel joint.
 */
declare function b2WheelJoint_GetUpperLimit(jointId: b2JointId): number;

/**
 * Sets new translation limits for a wheel joint. The function automatically orders
 * the limits so that the lower value is always less than or equal to the upper value.
 * When the limits change, the joint's impulses are reset to zero.
 * @param jointId - The identifier for the wheel joint.
 * @param lower - The lower translation limit.
 * @param upper - The upper translation limit.
 */
declare function b2WheelJoint_SetLimits(jointId: b2JointId, lower: number, upper: number): void;

/**
 * Enables or disables the motor on a wheel joint. When the motor state changes,
 * the motor impulse is reset to zero.
 * @param jointId - The identifier for the wheel joint
 * @param enableMotor - True to enable the motor, false to disable it
 */
declare function b2WheelJoint_EnableMotor(jointId: b2JointId, enableMotor: boolean): void;

/**
 * @param jointId - The identifier for the wheel joint to check.
 * @returns True if the motor is enabled, false otherwise.
 */
declare function b2WheelJoint_IsMotorEnabled(jointId: b2JointId): boolean;

/**
 * @param jointId - The identifier for the wheel joint to modify.
 * @param motorSpeed - The desired motor speed in radians per second.
 */
declare function b2WheelJoint_SetMotorSpeed(jointId: b2JointId, motorSpeed: number): void;

/**
 * Gets the motor speed of a wheel joint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The current motor speed of the wheel joint in radians per second.
 */
declare function b2WheelJoint_GetMotorSpeed(jointId: b2JointId): number;

/**
 * Calculates the motor torque by multiplying the motor impulse by the inverse
 * of the time step. The joint must be of type b2_wheelJoint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The current motor torque normalized by the step time (N⋅m).
 */
declare function b2WheelJoint_GetMotorTorque(jointId: b2JointId): number;

/**
 * Sets the maximum torque that can be applied by the wheel joint's motor.
 * The joint must be of type b2_wheelJoint.
 * @param jointId - The identifier for the wheel joint.
 * @param torque - The maximum motor torque value to set.
 */
declare function b2WheelJoint_SetMaxMotorTorque(jointId: b2JointId, torque: number): void;

/**
 * Gets the maximum motor torque of a wheel joint.
 * @param jointId - The identifier for the wheel joint.
 * @returns The maximum motor torque value.
 */
declare function b2WheelJoint_GetMaxMotorTorque(jointId: b2JointId): number;

declare namespace World { }

declare var b2_worlds: any;

/**
 * Initializes a global array of Box2D world instances if it hasn't been created yet.
 * Creates B2_MAX_WORLDS number of world instances and marks them as not in use.
 * If the array already exists, the function returns without doing anything.
 */
declare function b2CreateWorldArray(): void;

/**
 * Creates and initializes a new Box2D physics world with the specified parameters.
 * The function allocates memory for physics entities (bodies, joints, contacts),
 * initializes contact registers, creates necessary pools and arrays, and sets up
 * the world properties according to the provided definition.
 * @param def - World definition object containing initialization parameters including:
 * gravity, hitEventThreshold, restitutionThreshold, maximumLinearVelocity,
 * contactPushoutVelocity, contactHertz, contactDampingRatio, jointHertz,
 * jointDampingRatio, enableSleep, enableContinuous
 * @returns A world identifier object containing:
 * - index: number (worldId + 1)
 * - revision: number (world revision number)
 */
declare function b2CreateWorld(def: b2WorldDef): b2WorldId;

/**
 * Destroys a Box2D world instance and all its associated resources, including debug sets,
 * task contexts, event arrays, chains, bodies, shapes, contacts, joints, islands,
 * solver sets, constraint graph, broad phase, ID pools, and stack allocator.
 * Creates a new empty world instance with an incremented revision number.
 * @param worldId - The ID of the world to destroy
 */
declare function b2DestroyWorld(worldId: b2WorldId): void;

/**
 * Performs one step of physics simulation by updating broad phase pairs,
 * handling collisions, and solving the physics constraints. The function
 * manages contact events, sensor events, and body movement events during
 * the simulation step. It also handles warm starting and enforces velocity
 * limits based on world settings.
 * @param worldId - The identifier of the physics world to step
 * @param timeStep - The time increment to advance the simulation (in seconds)
 * @param subStepCount - The number of sub-steps to use for the iteration
 */
declare function b2World_Step(worldId: b2WorldId, timeStep: number, subStepCount: number): void;

/**
 * Renders debug visualization of a Box2D world, including shapes, joints, AABBs,
 * mass centers, and contact points based on the debug draw flags.
 * @param worldId - ID of the Box2D world to render
 * @param draw - Debug drawing context with rendering flags and methods
 */
declare function b2World_Draw(worldId: b2WorldId, draw: b2DebugDraw): void;

/**
 * Retrieves the body movement events from a Box2D world. Returns an empty events object
 * if the world is locked. The function copies the world's body move event array and count
 * into a new b2BodyEvents object.
 * @param worldId - The identifier for the Box2D world instance
 * @returns An object containing an array of body move events and their count
 */
declare function b2World_GetBodyEvents(worldId: b2WorldId): b2BodyEvents;

/**
 * Retrieves the sensor events from a Box2D world. The function returns both begin and end
 * sensor events that occurred during the simulation step. If the world is locked, it returns
 * an empty events object.
 * @param worldId - The identifier of the Box2D world
 * @returns An object containing arrays of sensor begin and end events
 */
declare function b2World_GetSensorEvents(worldId: b2WorldId): b2SensorEvents;

/**
 * Returns a b2ContactEvents object containing three arrays: contactBeginArray,
 * contactEndArray, and contactHitArray, representing different types of contact
 * events that occurred in the physics simulation. The object also includes
 * count values for each type of event.
 * @param worldId - The identifier of the Box2D world.
 * @returns An object containing arrays of begin, end, and hit contact events,
 * along with their respective counts.
 */
declare function b2World_GetContactEvents(worldId: b2WorldId): b2ContactEvents;

/**
 * Checks if a world ID is valid by verifying:
 * 1. The ID is not undefined
 * 2. The index is within valid bounds (1 to B2_MAX_WORLDS)
 * 3. The world exists at the specified index
 * 4. The revision number matches the world's current revision
 * @param id - The world identifier to validate, containing index1 and revision properties
 * @returns True if the world ID is valid and matches the stored world revision, false otherwise
 */
declare function b2World_IsValid(id: b2WorldId): boolean;

/**
 * Performs validation checks on a body ID including:
 * - Verifies the ID is defined and is a b2BodyId instance
 * - Checks the world index is within valid bounds
 * - Confirms the world exists and matches the ID
 * - Validates the body index exists in the world
 * - Ensures the body is active and the revision number matches
 * @param id - The body ID to validate
 * @returns True if the ID is valid and references an existing body, false otherwise
 */
declare function b2Body_IsValid(id: b2BodyId): boolean;

/**
 * Checks if a shape ID is valid by verifying:
 * - The ID exists and is not undefined
 * - The world index is within bounds
 * - The referenced world exists and matches the world ID
 * - The shape index is within bounds of the world's shape array
 * - The shape exists and is not marked as null
 * - The shape revision matches the ID's revision
 * @param id - The shape ID to validate, containing world0, index1, and revision properties
 * @returns True if the shape ID is valid and references an existing shape, false otherwise
 */
declare function b2Shape_IsValid(id: b2ShapeId): boolean;

/**
 * Checks if a chain ID is valid by verifying:
 * - The ID exists
 * - The world index is within valid bounds
 * - The world exists and matches the ID
 * - The chain index is within valid bounds
 * - The chain exists and is not null
 * - The revision number matches
 * @param id - The chain identifier containing world0 (world index),
 * index1 (chain index), and revision properties
 * @returns Returns true if the chain ID is valid and matches the current revision,
 * false otherwise
 */
declare function b2Chain_IsValid(id: b2ChainId): boolean;

/**
 * Checks if a joint ID is valid by verifying:
 * - The world index is within bounds
 * - The referenced world exists and matches the ID
 * - The joint index is within bounds
 * - The joint exists and is not null
 * - The revision number matches the joint's revision
 * @param id - The joint ID to validate, containing world0 (world index),
 * index1 (joint index), and revision properties
 * @returns True if the joint ID is valid and references an existing joint,
 * false otherwise
 */
declare function b2Joint_IsValid(id: b2JointId): boolean;

/**
 * Enables or disables the sleep management system for the specified Box2D world.
 * When sleep is disabled, all sleeping bodies are awakened. The function has no effect
 * if the world is locked or if the requested sleep state matches the current state.
 * @param worldId - The identifier of the Box2D world.
 * @param flag - When true, enables sleep management. When false, wakes all sleeping bodies.
 */
declare function b2World_EnableSleeping(worldId: b2WorldId, flag: boolean): void;

/**
 * Enables or disables warm starting for the specified Box2D world. Warm starting
 * cannot be modified while the world is locked. If the world is locked when this
 * function is called, the function will return without making any changes.
 * @param worldId - The identifier for the Box2D world instance
 * @param flag - Boolean value to enable or disable warm starting
 */
declare function b2World_EnableWarmStarting(worldId: b2WorldId, flag: boolean): void;

/**
 * Sets the continuous collision detection state for the specified Box2D world.
 * The function will not execute if the world is currently locked.
 * @param worldId - The identifier of the Box2D world.
 * @param flag - True to enable continuous collision detection, false to disable it.
 */
declare function b2World_EnableContinuous(worldId: b2WorldId, flag: boolean): void;

/**
 * Sets the restitution threshold for collision response in the specified Box2D world.
 * The value is clamped between 0 and Number.MAX_VALUE. The function will not execute
 * if the world is locked.
 * @param worldId - The identifier for the Box2D world.
 * @param value - The restitution threshold value to set.
 */
declare function b2World_SetRestitutionThreshold(worldId: b2WorldId, value: number): void;

/**
 * Sets the hit event threshold for a Box2D world. The value is clamped between 0 and Number.MAX_VALUE.
 * The function will not execute if the world is locked.
 * @param worldId - The identifier for the Box2D world instance
 * @param value - The new hit event threshold value to set
 */
declare function b2World_SetHitEventThreshold(worldId: b2WorldId, value: number): void;

/**
 * Sets three contact-related parameters for physics simulation: the constraint frequency (hertz),
 * damping ratio, and push-out velocity. All input parameters are clamped between 0 and MAX_VALUE.
 * The function will not execute if the world is locked.
 * @param worldId - The identifier for the Box2D world.
 * @param hertz - The frequency for contact constraint solving.
 * @param dampingRatio - The damping ratio for contact constraint solving.
 * @param pushOut - The velocity used for contact separation.
 */
declare function b2World_SetContactTuning(worldId: b2WorldId, hertz: number, dampingRatio: number, pushOut: number): void;

/**
 * This function is a stub that displays a warning message indicating that memory statistics
 * dumping is not supported in the Phaser Box2D JavaScript implementation.
 * @param worldId - The identifier of the Box2D world
 */
declare function b2World_DumpMemoryStats(worldId: b2WorldId): void;

/**
 * Performs a broadphase query using the world's dynamic tree to find all fixtures
 * that overlap with the given AABB. For each overlapping fixture that passes the
 * filter, the callback function is invoked.
 * @param worldId - The ID of the physics world to query
 * @param aabb - The axis-aligned bounding box defining the query region
 * @param filter - Filter settings to control which fixtures are included
 * @param fcn - Callback function that receives each overlapping fixture
 * @param context - User context data passed to the callback function
 */
declare function b2World_OverlapAABB(worldId: b2WorldId, aabb: b2AABB, filter: b2QueryFilter, fcn: b2OverlapResultFcn, context: any): void;

/**
 * Performs broad-phase AABB queries to find potential overlaps between the given circle
 * and all shapes in the world that match the filter criteria. For each potential overlap,
 * the callback function is invoked with the overlap details.
 * @param worldId - The ID of the physics world
 * @param circle - The circle shape to test for overlaps
 * @param transform - The position and rotation transform of the circle
 * @param filter - Filtering options for the overlap test
 * @param fcn - Callback function that handles overlap results
 * @param context - User context data passed to the callback function
 */
declare function b2World_OverlapCircle(worldId: b2WorldId, circle: b2Circle, transform: b2Transform, filter: b2QueryFilter, fcn: (...params: any[]) => any, context: any): void;

/**
 * Tests a capsule shape against all shapes in the world that pass the filter criteria.
 * For each potential overlap, calls the provided callback function.
 * Uses a broad phase tree structure to efficiently find potential overlaps.
 * @param worldId - The ID of the physics world
 * @param capsule - The capsule shape to test for overlaps
 * @param transform - The position and rotation of the capsule
 * @param filter - Filtering options for the query
 * @param fcn - Callback function that handles overlap results
 * @param context - User context data passed to the callback function
 */
declare function b2World_OverlapCapsule(worldId: b2WorldId, capsule: b2Capsule, transform: b2Transform, filter: b2QueryFilter, fcn: b2OverlapResultFcn, context: any): void;

/**
 * Performs overlap queries for a polygon shape against all shapes in the physics world
 * that match the provided filter criteria.
 * @param worldId - The identifier for the physics world
 * @param polygon - The polygon shape to test for overlaps
 * @param transform - The position and rotation of the polygon
 * @param filter - Filtering criteria for the overlap test
 * @param fcn - Callback function to handle overlap results
 * @param context - User data passed to the callback function
 */
declare function b2World_OverlapPolygon(worldId: b2WorldId, polygon: b2Polygon, transform: b2Transform, filter: b2QueryFilter, fcn: b2OverlapResultFcn, context: void): void;

/**
 * Performs a ray cast operation in the physics world to detect intersections between
 * a ray and physics bodies.
 * @param worldId - The ID of the physics world
 * @param origin - The starting point of the ray
 * @param translation - The direction and length of the ray
 * @param filter - Filtering options for the ray cast
 * @param fcn - Callback function to handle ray cast results
 * @param context - User context data passed to the callback
 */
declare function b2World_CastRay(worldId: b2WorldId, origin: b2Vec2, translation: b2Vec2, filter: b2QueryFilter, fcn: b2CastResultFcn, context: void): b2TreeStats;

/**
 * Casts a ray through the physics world and returns information about the closest
 * intersection. The ray is defined by an origin point and a translation vector.
 * The operation checks all body types in the broad phase using a dynamic tree
 * structure.
 * @param worldId - The identifier for the physics world
 * @param origin - The starting point of the ray
 * @param translation - The direction and length of the ray
 * @param filter - Filter settings for the ray cast
 * @returns Information about the closest intersection found
 */
declare function b2World_CastRayClosest(worldId: b2WorldId, origin: b2Vec2, translation: b2Vec2, filter: b2QueryFilter): b2RayResult;

/**
 * Casts a circle shape through the physics world from its initial position
 * along a translation vector, detecting collisions with other shapes. The cast
 * is performed against all body types in the broad phase, stopping if a collision
 * occurs at fraction 0.
 * @param worldId - The identifier for the physics world
 * @param circle - The circle shape to cast
 * @param originTransform - The initial transform of the circle
 * @param translation - The displacement vector for the cast
 * @param filter - Filtering options for the cast
 * @param fcn - Callback function to handle cast results
 * @param context - User data passed to the callback function
 */
declare function b2World_CastCircle(worldId: b2WorldId, circle: b2Circle, originTransform: b2Transform, translation: b2Vec2, filter: b2QueryFilter, fcn: b2CastResultFcn, context: void): void;

/**
 * Performs a shape cast of a capsule through the physics world, detecting collisions
 * along the specified translation path.
 * @param worldId - The identifier for the physics world
 * @param capsule - The capsule shape to cast
 * @param originTransform - The initial transform of the capsule, containing position (p) and rotation (q)
 * @param translation - The translation vector defining the cast path
 * @param filter - Filter settings for the cast operation
 * @param fcn - Callback function to handle cast results
 * @param context - User context data passed to the callback function
 */
declare function b2World_CastCapsule(worldId: b2WorldId, capsule: b2Capsule, originTransform: b2Transform, translation: b2Vec2, filter: b2QueryFilter, fcn: b2CastResultFcn, context: void): void;

/**
 * Performs a shape cast of a polygon through the physics world, detecting collisions along the way.
 * @param worldId - ID of the physics world
 * @param polygon - The polygon shape to cast
 * @param originTransform - Initial transform of the polygon, containing position and rotation
 * @param translation - The displacement vector to cast the polygon along
 * @param filter - Filter to determine which fixtures to check against
 * @param fcn - Callback function to handle cast results
 * @param context - User context data passed to the callback function
 */
declare function b2World_CastPolygon(worldId: b2WorldId, polygon: b2Polygon, originTransform: b2Transform, translation: b2Vec2, filter: b2QueryFilter, fcn: b2CastResultFcn, context: any): void;

/**
 * Sets a callback function that is invoked before the physics solver runs.
 * The callback receives the provided context pointer when executed.
 * @param worldId - The identifier for the Box2D world instance
 * @param fcn - The pre-solve callback function to be executed
 * @param context - User data pointer passed to the callback function
 */
declare function b2World_SetPreSolveCallback(worldId: b2WorldId, fcn: b2PreSolveFcn, context: void): void;

/**
 * Sets a custom filter callback function for a Box2D world instance. The callback
 * can be used to implement custom collision filtering logic. The context parameter
 * allows passing additional data to the callback function.
 * @param worldId - The identifier for the Box2D world instance.
 * @param fcn - The custom filter callback function.
 * @param context - A pointer to user-defined context data.
 */
declare function b2World_SetCustomFilterCallback(worldId: b2WorldId, fcn: b2CustomFilterFcn, context: void): void;

/**
 * Updates the gravity vector of the specified Box2D world. The gravity vector
 * defines the global acceleration applied to all dynamic bodies in the world.
 * @param worldId - The identifier for the Box2D world instance.
 * @param gravity - The gravity vector to apply to the world. Defaults to (0,0).
 */
declare function b2World_SetGravity(worldId: b2WorldId, gravity: b2Vec2): void;

/**
 * Retrieves the current gravity vector from the specified Box2D world instance.
 * The gravity vector represents the global gravity force applied to all dynamic bodies in the world.
 * @param worldId - The identifier for the Box2D world instance.
 * @returns The gravity vector of the world. A 2D vector with x and y components.
 */
declare function b2World_GetGravity(worldId: b2WorldId): b2Vec2;

/**
 * Creates a circular explosion centered at the given position that applies radial forces
 * to dynamic bodies within the explosion radius. The explosion force decreases with
 * distance from the center point.
 * @param worldId - The ID of the Box2D world
 * @param position - The center point of the explosion
 * @param radius - The radius of the explosion effect
 * @param magnitude - The force magnitude of the explosion
 */
declare function b2World_Explode(worldId: b2WorldId, position: b2Vec2, radius: number, magnitude: number): void;

