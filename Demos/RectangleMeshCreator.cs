// using System;
// using System.Numerics;
// using BepuPhysics.Collidables;
// using BepuUtilities.Memory;
// using BepuUtilities;
//
// namespace Demos;
//
// public static class RectangleMeshCreator
// {
//     public static Mesh CreateRectangleMesh(float width, float height, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
//     {
//         // Create rectangle with 2 triangles
//         pool.Take<Triangle>(2, out var triangles);
//
//         float halfWidth = width * 0.5f;
//         float halfHeight = height * 0.5f;
//
//         // Define vertices for rectangle
//         Vector3 v0 = new Vector3(-halfWidth, -halfHeight, 0); // Bottom-left
//         Vector3 v1 = new Vector3(halfWidth, -halfHeight, 0);  // Bottom-right
//         Vector3 v2 = new Vector3(-halfWidth, halfHeight, 0);  // Top-left
//         Vector3 v3 = new Vector3(halfWidth, halfHeight, 0);   // Top-right
//
//         // First triangle (bottom-left, bottom-right, top-left)
//         ref var triangle0 = ref triangles[0];
//         triangle0.A = v0;
//         triangle0.B = v1;
//         triangle0.C = v2;
//
//         // Second triangle (bottom-right, top-right, top-left)
//         ref var triangle1 = ref triangles[1];
//         triangle1.A = v1;
//         triangle1.B = v3;
//         triangle1.C = v2;
//
//         return new Mesh(triangles, scaling, pool, dispatcher);
//     }
//
//     public static Mesh[] CreateThickRectangleMesh(float thickness, int numberOfLayers, float skewedness, float width, float height, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
//     {
//         var meshes = new Mesh[numberOfLayers];
//
//         if (numberOfLayers == 1)
//         {
//             meshes[0] = CreateRectangleMesh(width, height, scaling, pool, dispatcher);
//         }
//         else
//         {
//             float layerSpacing = thickness / (numberOfLayers - 1);
//
//             for (int i = 0; i < numberOfLayers; i++)
//             {
//                 // Calculate size change based on skewedness
//                 float layerProgress = (float)i / (numberOfLayers - 1); // 0 to 1
//                 float sizeMultiplier = 1.0f + (skewedness * layerProgress);
//
//                 float currentWidth = width * sizeMultiplier;
//                 float currentHeight = height * sizeMultiplier;
//
//                 meshes[i] = CreateRectangleMesh(currentWidth, currentHeight, scaling, pool, dispatcher);
//             }
//         }
//
//         return meshes;
//     }
// }

using System.Numerics;
using BepuPhysics.Collidables;
using BepuUtilities.Memory;
using BepuUtilities;

namespace Demos;

public static class RectangleMeshCreator
{
    public static Mesh CreateRectangleMesh(float width, float height, float zOffset, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
    {
        // Create rectangle with 2 triangles
        pool.Take<Triangle>(2, out var triangles);

        float halfWidth = width * 0.5f;
        float halfHeight = height * 0.5f;

        // Define vertices for rectangle
        Vector3 v0 = new Vector3(-halfWidth, -halfHeight, zOffset); // Bottom-left
        Vector3 v1 = new Vector3(halfWidth, -halfHeight, zOffset);  // Bottom-right
        Vector3 v2 = new Vector3(-halfWidth, halfHeight, zOffset);  // Top-left
        Vector3 v3 = new Vector3(halfWidth, halfHeight, zOffset);   // Top-right

        // First triangle (bottom-left, bottom-right, top-left)
        ref var triangle0 = ref triangles[0];
        triangle0.A = v0;
        triangle0.B = v1;
        triangle0.C = v2;

        // Second triangle (bottom-right, top-right, top-left)
        ref var triangle1 = ref triangles[1];
        triangle1.A = v1;
        triangle1.B = v3;
        triangle1.C = v2;

        return new Mesh(triangles, scaling, pool, dispatcher);
    }

    public static Mesh[] CreateThickRectangleMesh(float thickness, int numberOfLayers, float skewedness, float width, float height, Vector3 scaling, BufferPool pool, IThreadDispatcher dispatcher = null)
    {
        var meshes = new Mesh[numberOfLayers];

        if (numberOfLayers == 1)
        {
            meshes[0] = CreateRectangleMesh(width, height, 0, scaling, pool, dispatcher);
        }
        else
        {
            float layerSpacing = thickness / (numberOfLayers - 1);

            for (int i = 0; i < numberOfLayers; i++)
            {
                // Calculate Z position for this layer
                float zOffset = -(thickness * 0.5f) + (i * layerSpacing);

                // Calculate size change based on skewedness
                float layerProgress = (float)i / (numberOfLayers - 1); // 0 to 1
                float sizeMultiplier = 1.0f + (skewedness * layerProgress);

                float currentWidth = width * sizeMultiplier;
                float currentHeight = height * sizeMultiplier;

                meshes[i] = CreateRectangleMesh(currentWidth, currentHeight, zOffset, scaling, pool, dispatcher);
            }
        }

        return meshes;
    }
}
