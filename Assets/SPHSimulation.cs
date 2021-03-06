﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

public class SPHSimulation : MonoBehaviour
{
    [SerializeField] private GameObject sphere;
    private StreamReader File;
    private string text;
    private GameObject[] particles;
    private int frame = 0;
    public int particleCount;
    private float timer = 0.0f;
    [SerializeField] public int frameCount;
    float fps = 1 / 15;


    // Start is called before the first frame update
    void Start()
    {

        particles = new GameObject[particleCount];
        for(int i = 0; i<particles.Length; i++)
        {
            particles[i] = Instantiate(sphere);
        }

        File = new StreamReader("Assets/Positions/DamBreak" + frame.ToString() + ".txt");
        text = File.ReadToEnd();
        string[] xyz = text.Split(',');
        for (long i = 0; i < particles.Length; i++)
        {
            particles[i].transform.position = new Vector3(float.Parse(xyz[i * 3]), float.Parse(xyz[i * 3 + 1]), float.Parse(xyz[i * 3 + 2]));
        }
        frame++;

    }

    // Update is called once per frame
    void Update()
    {
        if (frame < frameCount)
        {
            timer += Time.deltaTime;
            if (timer >= fps)
            {
                File = new StreamReader("Assets/Positions/DamBreak" + frame.ToString() + ".txt");
                text = File.ReadToEnd();
                string[] xyz = text.Split(',');
                for (long i = 0; i < particles.Length; i++)
                {
                    particles[i].transform.position = new Vector3(float.Parse(xyz[i * 3]), float.Parse(xyz[i * 3 + 1]), float.Parse(xyz[i * 3 + 2]));
                }
                frame++;

                timer = 0.0f;
            }
        }
        else
        {
            frame = 1;
        }
    }
}
