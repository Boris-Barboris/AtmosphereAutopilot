using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;

namespace AtmosphereAutopilot
{

    /// <summary>
    /// Background thread, designed to execute a set of functions with limits on execution freuency
    /// </summary>
    public class BackgroundThread : IDisposable
    {
        int idle_pause;

        List<Func<bool>> execution_que = new List<Func<bool>>();

        Thread thread;

        public BackgroundThread(int idle_pause = 1)
        {
            this.idle_pause = idle_pause;
            thread = new Thread(new ThreadStart(cycle));
        }

        public void Start()
        {
            if (abort)
                throw new InvalidOperationException("Thread was aborted");
            if (!thread.IsAlive)
                thread.Start();
        }

        public void Stop()
        {
            abort = true;
        }

        public void Dispose()
        {
            Stop();
        }

        public bool IsRunning { get { return thread.IsAlive; } }

        bool abort = false;

        void cycle()
        {
            bool success = false;
            foreach (var func in execution_que)
            {
                if (abort)
                    return;
                success |= func();
            }
            if (abort)
                return;
            if (!success)
                Thread.Sleep(idle_pause);
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

        List<Func<bool>> removal_que = new List<Func<bool>>();
        List<Func<bool>> addition_que = new List<Func<bool>>();

        public void remove_func(Func<bool> hndl)
        {
            lock (removal_que)
                removal_que.Add(hndl);
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
        }
    }
}
