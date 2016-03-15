/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading;
//using UnityEngine;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Background thread, designed to execute a set of functions with limits on execution freuency
    /// </summary>
    public class BackgroundThread : IDisposable
    {
        List<Func<bool>> execution_que = new List<Func<bool>>();

        Thread thread;
        ManualResetEvent sema = new ManualResetEvent(true);

        TextWriter logger;

        public BackgroundThread(string log_folder_name)
        {
            thread = new Thread(new ThreadStart(cycle));
            logger = File.CreateText(log_folder_name + "/thread.log");
        }

        public void Start()
        {
            if (stop)
                throw new InvalidOperationException("Thread was aborted");
            if (!thread.IsAlive)
            {
                thread.Start();
                logger.WriteLine("Starting");
                logger.Flush();
            }
        }

        public void Stop()
        {
            Resume();
            stop = true;
            lock (logger)
            {
                logger.WriteLine("Stopping");
                logger.Close();
            }
        }

        public void Dispose()
        {
            Stop();
        }

        public void Pause()
        {
            if (stop)
                throw new InvalidOperationException("Thread was stopped");
            sema.Reset();
        }

        public void Resume()
        {
            if (stop)
                throw new InvalidOperationException("Thread was stopped");
            sema.Set();
        }

        public bool IsRunning { get { return thread.IsAlive; } }

        bool stop = false;

        void cycle()
        {
            while (true)
            {
                sema.WaitOne();
                bool success = false;
                foreach (var func in execution_que)
                {
                    if (stop)
                        return;
                    try
                    {
                        success |= func();
                    }
                    catch (Exception e)
                    {
                        lock (logger)
                        {
                            logger.WriteLine("Background thread exception - broken function removed from queue");
                            logger.WriteLine(e.Message);
                            logger.WriteLine(e.StackTrace);
                            logger.WriteLine();
                            logger.Flush();
                        }
                        remove_func(func);
                    }
                    sema.WaitOne();
                }
                if (stop)
                    return;
                if (!success)
                    Thread.Sleep(5);
                if (que_flag)
                {
                    que_flag = false;
                    lock (removal_que)
                    {
                        // Process removal requests
                        while (removal_que.Count > 0)
                        {
                            var hndl = removal_que[0];
                            execution_que.Remove(hndl);
                            removal_que.RemoveAt(0);
                        }
                    }
                    lock (addition_que)
                    {
                        // Process addition requests
                        while (addition_que.Count > 0)
                        {
                            var hndl = addition_que[0];
                            execution_que.Add(hndl);
                            addition_que.RemoveAt(0);
                        }
                    }
                }
            }
        }

        List<Func<bool>> removal_que = new List<Func<bool>>();
        List<Func<bool>> addition_que = new List<Func<bool>>();
        volatile bool que_flag = false;

        /// <summary>
        /// Remove function from thread execution queue.
        /// </summary>
        /// <param name="hndl">Function to remove,</param>
        public void remove_func(Func<bool> hndl)
        {
            lock (removal_que)
                removal_que.Add(hndl);
            que_flag = true;
        }

        /// <summary>
        /// Add function to thread execution queue
        /// </summary>
        /// <param name="hndl">Handle to parameterless function, that returns true if it was fully executed or false if
        /// it didn't execute. You would want to return false if you didn't allow it to run now.</param>
        public void add_func(Func<bool> hndl)
        {
            lock (addition_que)
                addition_que.Add(hndl);
            que_flag = true;
        }
    }
}
