using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public delegate void NearbyPointCallback(int pointIndex, Vector3 point);
public interface IPointHash
{
    void Build(List<Vector3> points);
    void ForEachNearbyPoint(Vector3 origin, float radius, NearbyPointCallback callbackAction);
}

public class PointHashSearcher : IPointHash
{
    public delegate void NearbyPointCallback(int pointIndex, Vector3 point);

    private readonly float gridSpacing = 1.0f;
    private readonly Vector3Int resolution = Vector3Int.one;
    private List<Vector3> points;
    private Dictionary<int, List<int>> buckets;

    public PointHashSearcher(Vector3Int resolution, float gridSpacing)
    {
        this.gridSpacing = gridSpacing;
        this.resolution = resolution;

        buckets = new Dictionary<int, List<int>>();
        points = new List<Vector3>();
    }

    public void Build(List<Vector3> points)
    {
        buckets.Clear();
        this.points.Clear();

        if (points.Count == 0)
            return;

        //this.buckets.Resize(resolution.x * resolution.y * resolution.z, new List<int>());
        this.points.Resize(points.Count);

        //put points into buckets

        for (int i = 0; i < points.Count; i++)
        {
            this.points[i] = points[i];
            int key = getHashKeyFromPosition(points[i]);

            if (!this.buckets.ContainsKey(key))
                this.buckets.Add(key, new List<int>());

            this.buckets[key].Add(i);
        }
    }

    public void ForEachNearbyPoint(Vector3 origin, float radius, global::NearbyPointCallback callbackAction)
    {
        if (this.buckets.Count == 0)
        {
            return;
        }

        int[] nearbyKeys = new int[8];
        getNearbyKeys(origin, ref nearbyKeys);

        float queryRadiusSquared = radius * radius;

        for (int i = 0; i < 8; i++)
        {
            int nearbyKey = nearbyKeys[i];
            if (!buckets.ContainsKey(nearbyKey))
                continue;

            var bucket = this.buckets[nearbyKey];
            int numberOfPointsInBucket = bucket.Count;

            for (int j = 0; j < numberOfPointsInBucket; j++)
            {
                int pointIndex = bucket[j];
                Vector3 point = points[pointIndex];
                float radiusSquared = (point - origin).sqrMagnitude;
                if (radiusSquared <= queryRadiusSquared)
                {
                    callbackAction(pointIndex, point);
                }
            }
        }
    }

    private void getNearbyKeys(Vector3 origin, ref int[] nearbyKeys)
    {
        Vector3Int originIndex = getBucketIndex(origin);
        Vector3Int[] nearbyBucketIndices = new Vector3Int[8];

        for (int i = 0; i < 8; i++)
        {
            nearbyBucketIndices[i] = originIndex;
        }


        if ((originIndex.x + 0.5f) * gridSpacing <= origin.x)
        {
            nearbyBucketIndices[4].x += 1; nearbyBucketIndices[5].x += 1; nearbyBucketIndices[6].x += 1; nearbyBucketIndices[7].x += 1;
        }
        else
        {
            nearbyBucketIndices[4].x -= 1; nearbyBucketIndices[5].x -= 1; nearbyBucketIndices[6].x -= 1; nearbyBucketIndices[7].x -= 1;
        }
        if ((originIndex.y + 0.5f) * gridSpacing <= origin.y)
        {
            nearbyBucketIndices[2].y += 1; nearbyBucketIndices[3].y += 1; nearbyBucketIndices[6].y += 1; nearbyBucketIndices[7].y += 1;
        }
        else
        {
            nearbyBucketIndices[2].y -= 1; nearbyBucketIndices[3].y -= 1; nearbyBucketIndices[6].y -= 1; nearbyBucketIndices[7].y -= 1;
        }
        if ((originIndex.z + 0.5f) * gridSpacing <= origin.z)
        {
            nearbyBucketIndices[1].z += 1; nearbyBucketIndices[3].z += 1; nearbyBucketIndices[5].z += 1; nearbyBucketIndices[7].z += 1;
        }
        else
        {
            nearbyBucketIndices[1].z -= 1; nearbyBucketIndices[3].z -= 1; nearbyBucketIndices[5].z -= 1; nearbyBucketIndices[7].z -= 1;
        }


        for (int i = 0; i < 8; i++)
        {
            nearbyKeys[i] = getHashKeyFromBucketIndex(nearbyBucketIndices[i]);
        }

    }

    Vector3Int getBucketIndex(Vector3 position)
    {
        Vector3Int bucketIndex = new Vector3Int(
            (int)Mathf.Floor(position.x / gridSpacing),
            (int)Mathf.Floor(position.y / gridSpacing),
            (int)Mathf.Floor(position.z / gridSpacing));

        return bucketIndex;
    }

    private int getHashKeyFromPosition(Vector3 position)
    {
        Vector3Int bucketIndex = getBucketIndex(position);

        return (bucketIndex.z * bucketIndex.y) + bucketIndex.x;
        //return getHashKeyFromBucketIndex(bucketIndex);
    }

    private int getHashKeyFromBucketIndex(Vector3Int bucketIndex)
    {
        Vector3Int wrappedIndex = bucketIndex;

        wrappedIndex.x = bucketIndex.x % resolution.x;
        wrappedIndex.x = bucketIndex.y % resolution.y;
        wrappedIndex.x = bucketIndex.z % resolution.z;

        if (wrappedIndex.x < 0) { wrappedIndex.x += resolution.x; }
        if (wrappedIndex.y < 0) { wrappedIndex.y += resolution.y; }
        if (wrappedIndex.z < 0) { wrappedIndex.z += resolution.z; }

        return (wrappedIndex.z * resolution.y + wrappedIndex.y) * resolution.x + wrappedIndex.x;
    }
}
