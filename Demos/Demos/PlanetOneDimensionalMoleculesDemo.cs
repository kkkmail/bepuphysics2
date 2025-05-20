using BepuPhysics;
using BepuPhysics.Collidables;
using BepuPhysics.Constraints;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;
using DemoRenderer.UI;
using DemoUtilities;
using System;
using System.Numerics;

namespace Demos.Demos;


/// <summary>
/// Shows how to use custom velocity integration to implement planetary gravity.
/// </summary>
public class PlanetOneDimensionalMoleculesDemo : Demo
{
    struct PlanetaryGravityCallbacks : IPoseIntegratorCallbacks
    {
        public Vector3 PlanetCenter;
        public float Gravity;

        public readonly AngularIntegrationMode AngularIntegrationMode => AngularIntegrationMode.Nonconserving;

        public readonly bool AllowSubstepsForUnconstrainedBodies => false;

        public readonly bool IntegrateVelocityForKinematics => false;

        public void Initialize(Simulation simulation)
        {
        }

        float gravityDt;

        public void PrepareForIntegration(float dt)
        {
            //No point in repeating this for every body; cache it.
            gravityDt = dt * Gravity;
        }

        public void IntegrateVelocity(Vector<int> bodyIndices, Vector3Wide position, QuaternionWide orientation,
            BodyInertiaWide localInertia, Vector<int> integrationMask, int workerIndex, Vector<float> dt,
            ref BodyVelocityWide velocity)
        {
            var offset = position - Vector3Wide.Broadcast(PlanetCenter);
            var distance = offset.Length();
            velocity.Linear -= new Vector<float>(gravityDt) * offset /
                               Vector.Max(Vector<float>.One, distance * distance * distance);
        }
    }

    public override void Initialize(ContentArchive content, Camera camera)
    {
        camera.Position = new Vector3(-110, 80, -50);
        camera.Yaw = 0;
        camera.Pitch = MathF.PI * -0.5f;

        #region Parameters

        var frequency = 5.0f;
        var dampingRatio = -0.2625f;

        var maximumRecoveryVelocity = float.MaxValue;
        var frictionCoefficient = 0.0f;

        var gravityValue = 100000.0f;

        var orbiterRadius = 1.0f;
        var orbiterWidth = 0.3f;
        var orbiterMass = 1.0f;

        const int length = 40;
        const int width = 40;
        const int height = 40;

        var mainOrigin = new Vector3(-200, 300, 0);
        // var mainVelocity = new Vector3(30, 0, 0);
        var mainVelocity = new Vector3();
        var spacing = new Vector3(5);

        #endregion

        Simulation = Simulation.Create(BufferPool,
            new DemoNarrowPhaseCallbacks(
                new SpringSettings(frequency, dampingRatio),
                maximumRecoveryVelocity: maximumRecoveryVelocity,
                frictionCoefficient: frictionCoefficient),
            new PlanetaryGravityCallbacks { PlanetCenter = new Vector3(), Gravity = gravityValue },
            new SolveDescription(8, 1));

        Simulation.Statics.Add(new StaticDescription(new Vector3(), Simulation.Shapes.Add(new Sphere(50))));

        var orbiter = new Capsule(orbiterWidth, 2 * orbiterRadius);
        var inertia = orbiter.ComputeInertia(orbiterMass);
        var orbiterShapeIndex = Simulation.Shapes.Add(orbiter);

        for (var i = 0; i < length; ++i)
        {
            for (var j = 0; j < height; ++j)
            {
                var origin = mainOrigin + spacing * new Vector3(length * -0.5f, 0, width * -0.5f);
                for (var k = 0; k < width; ++k)
                {
                    Simulation.Bodies.Add(BodyDescription.CreateDynamic(
                        origin + new Vector3(i, j, k) * spacing, mainVelocity, inertia, orbiterShapeIndex, 0.01f));
                }
            }
        }
    }

    public override void Render(Renderer renderer, Camera camera, Input input, TextBuilder text, Font font)
    {
        var bottomY = renderer.Surface.Resolution.Y;
        renderer.TextBatcher.Write(
            text.Clear().Append("The library does not prescribe any particular kind of gravity."),
            new Vector2(16, bottomY - 48), 16, Vector3.One, font);
        renderer.TextBatcher.Write(
            text.Clear()
                .Append(
                    "The IPoseIntegratorCallbacks provided to the simulation is responsible for telling the simulation how to integrate."),
            new Vector2(16, bottomY - 32), 16, Vector3.One, font);
        renderer.TextBatcher.Write(
            text.Clear().Append("In this demo, all bodies are pulled towards the center of the planet."),
            new Vector2(16, bottomY - 16), 16, Vector3.One, font);
        base.Render(renderer, camera, input, text, font);
    }
}
