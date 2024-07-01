using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using System.Diagnostics;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine.Rendering.Universal;
using UnityEditor;
using UnityEngine.UIElements;
using System;
using UnityEngine.Jobs;
using FlowFieldNavigation;
using System.Runtime.CompilerServices;
namespace Assets
{

    public class test : MonoBehaviour
    {
        double normalms;
        double testms;
        [SerializeField] int size;
        private void Start()
        {
        }
        private void Update()
        {/*

            NativeArray<int> array = new NativeArray<int>(1000000, Allocator.Persistent);
            myarray<int> myarray = new myarray<int>(1000000, Allocator.Persistent);
            NativeReference<int> res = new NativeReference<int>(Allocator.Persistent);
            NativeArray<int> indicies = new NativeArray<int>(1000000, Allocator.Persistent);
            NativeArray<Index> myindicies = new NativeArray<Index>(1000000, Allocator.Persistent);
            for(int i = 0; i < indicies.Length; i++)
            {
                indicies[i] = i;
                myindicies[i] = new Index(i);
            }
            testjob tj = new testjob()
            {
                indicies = myindicies,
                array = myarray,
                res = res,
            };

            normaljob nj = new normaljob()
            {
                indicies = indicies,
                array = array,
                res = res,
            };

            Stopwatch sw = new Stopwatch();
            sw.Start();
            tj.Schedule().Complete();
            sw.Stop();
            testms += sw.Elapsed.TotalMilliseconds;

            sw = new Stopwatch();
            sw.Start();
            nj.Schedule().Complete();
            sw.Stop();
            normalms+= sw.Elapsed.TotalMilliseconds;

            array.Dispose();
            res.Dispose();
            myarray.Dispose();
            indicies.Dispose();
            myindicies.Dispose();
            UnityEngine.Debug.Log(testms / normalms);*/

        }
    }

    [BurstCompile]
    struct testjob : IJob
    {
        internal NativeArray<Index> indicies;
        internal myarray<int> array;
        internal NativeReference<int> res;
        public void Execute()
        {
            int sum = 0;
            for (int i = 0; i < array.Length; i++)
            {
                sum += array[indicies[i]];
            }
            res.Value = sum;
        }
    }

    [BurstCompile]
    struct normaljob : IJob
    {
        internal NativeArray<int> indicies;
        internal NativeArray<int> array;
        internal NativeReference<int> res;
        public void Execute()
        {
            int sum = 0;
            for (int i = 0; i < array.Length; i++)
            {
                sum += array[indicies[i]];
            }
            res.Value = sum;
        }
    }

    public struct myarray<T> where T : unmanaged
    {
        NativeArray<T> _array;

        public myarray(int size, Allocator allocator)
        {
            _array = new NativeArray<T>(size, allocator);
        }

        public T this[Index index]
        {
            get
            {
                return _array[index.index];
            }
            set
            {
                _array[index.index] = value;
            }
        }
        public int Length
        {
            get { return _array.Length; }
        }
        public void Dispose()
        {
            _array.Dispose();
        }

    }

    public struct Index
    {
        public int index;

        public Index(int idx) => index = idx;
    }
}

