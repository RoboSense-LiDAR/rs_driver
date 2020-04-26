  #include <chrono>
  
  inline double getTime(void)
  {
    const auto t = std::chrono::system_clock::now();
    const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
    return (double)t_sec.count();
  }