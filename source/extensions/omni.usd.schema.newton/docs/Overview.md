# Overview

The **omni.usd.schema.newton** extension defines USD schemas for Newton physics simulation, providing a comprehensive set of API schemas that extend standard USD physics primitives with Newton-specific simulation attributes. This extension integrates Newton's physics solvers into USD-based physics workflows by defining schemas for scene configuration, solver parameters, and enhanced material properties.

## Key Components

### Scene Configuration Schemas

**NewtonSceneAPI** provides the base configuration for Newton physics scenes, extending PhysicsScene prims with core simulation parameters. This schema controls fundamental aspects like solver iteration limits, simulation timestep frequency, and gravity enablement settings.

**NewtonXpbdSceneAPI** configures Newton's XPBD (eXtended Position-Based Dynamics) solver with specialized relaxation parameters for different constraint types including soft body dynamics, contact handling, and joint behavior. The schema provides fine-grained control over constraint solving through relaxation multipliers and compliance settings for both linear and angular constraints.

**NewtonKaminoSceneAPI** provides configuration for Newton's Kamino solver, featuring PADMM (Proximal Augmented Lagrangian Method) constraint solving with configurable tolerances for primal, dual, and complementarity residuals. This schema includes advanced solver features like warmstarting modes, Nesterov acceleration, and Baumgarte stabilization parameters for different constraint categories.

### Articulation and Body Schemas

**NewtonArticulationRootAPI** extends the standard PhysicsArticulationRootAPI with Newton-specific articulation control, particularly self-collision management across articulated body chains.

**NewtonCollisionAPI** enhances collision detection for geometric primitives by adding contact margin and contact gap parameters that control collision surface inflation and detection thresholds.

**NewtonMeshCollisionAPI** provides specialized mesh collision handling with convex hull approximation controls, allowing configuration of vertex limits for hull generation algorithms.

### Material and Joint Schemas

**NewtonMaterialAPI** extends standard physics materials with Newton-specific friction properties including torsional friction for spinning resistance and rolling friction for rolling motion dynamics.

**NewtonMimicAPI** implements joint mimic constraints that enforce mathematical relationships between joint degrees of freedom, enabling one joint to follow another with configurable scaling and offset parameters through the relationship `joint0 = coef0 + coef1 * joint1`.

## Integration

The extension loads early in the USD schema initialization process to ensure Newton schemas are available before other physics-related extensions attempt to use them. It depends on **omni.usd.libs** for core USD functionality and integrates with the standard USD physics schema hierarchy by extending PhysicsScene, PhysicsCollisionAPI, PhysicsMeshCollisionAPI, PhysicsMaterialAPI, PhysicsArticulationRootAPI, and PhysicsJoint schemas.

Each Newton schema inherits from its corresponding standard physics schema, ensuring compatibility with existing USD physics workflows while providing Newton-specific enhancements. The schemas apply to specific USD primitive types as defined in their plugInfo.json configuration, maintaining type safety and proper schema application rules.
