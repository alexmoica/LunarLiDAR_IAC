<a name="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/github_username/repo_name">
    <img src="logo.png" alt="Logo" width="80" height="80">
  </a>

<h3 align="center">LunarLiDAR</h3>
  <p align="center">
    SLAM navigation system for lunar rovers aiming to evaluate the advantages of utilizing 4D LIDAR in contrast to conventional 3D LIDAR in the lunar environment. By fusing simulated 4D LIDAR with integrated sensors, this system aims to enhance collision detection, avoidance capabilities, and overall navigation efficiency.
    <br />
  </p>
</div>


<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#results">Results & Findings</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

![4D LIDAR](https://github.com/alexmoica/LunarLiDAR_IAC/assets/6493847/c68510d8-2a79-4049-a02d-5773c9187135)

<p>The recent advance of 4D LIDAR (Light Detection and Ranging) technology has opened up a new set of possibilities to explore space in much greater detail. This innovative technology is based on the principle of the Doppler effect, which enables it to compute instant velocity in real-time and hence adds a fourth dimension to its preceding technology. In 2022, AEVA, a company specialized in the production of autonomous driving sensors, developed the 4D LIDAR, a new tool that will revolutionize autonomous driving. This new generation of lidars offers better performance in terms of depth, instant velocity, reflectivity, and vision.</p>

<p>In recent times, the exploration of the lunar surface and utilization of lunar resources has become a dominant agenda of several government space agencies and private companies. Furthermore, with the rapid growth in the New Space sector, organizations would continue to highly invest in lunar exploration. With an apparent increase in rover missions in the near future, the ability of the rover system to identify other moving rovers and compute its velocity would become a critical design parameter.</p>

<p>The purpose of this project is to study the performance of the 4D LIDAR, combined with other onboard components using sensor fusion, and how it can help achieve better results when covering an undiscovered area with unknown environmental parameters.</p>

<p>The project plans to integrate the 4D LIDAR technology with different onboard sensors, evaluate its performance theoretically using software simulation, and present the final data illustrating the overall results. This would help demonstrate the higher performance parameters and precision levels that could be achieved for future rover missions.</p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


### Built With

* [![ROS 2][ROS2-shield]][ROS2-url]
* [![Python][Python-shield]][Python-url]
* [![C][C-shield]][C-url]
* [![Bash][Bash-shield]][Bash-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
  ```sh
  npm install npm@latest -g
  ```

### Installation

1. Get a free API Key at [https://example.com](https://example.com)
2. Clone the repo
   ```sh
   git clone https://github.com/github_username/repo_name.git
   ```
3. Install NPM packages
   ```sh
   npm install
   ```
4. Enter your API in `config.js`
   ```js
   const API_KEY = 'ENTER YOUR API';
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- USAGE EXAMPLES -->
## Usage

To run the project, do these steps

workflow of project (action server, publisher, etc.)

3d video

4d video

From this quick test case, you can already see the performance advantages of 4d (screenshot)


<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- RESULTS & FINDINGS -->
## Results & Findings

<p align="right">(<a href="#readme-top">back to top</a>)</p>




<!-- CONTACT -->
## Contact

Alex Moica - <a href="https://www.linkedin.com/in/alexmoica/" target="_blank">LinkedIn</a> - alex@alexmoica.com

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
[ROS2-shield]: https://img.shields.io/badge/ROS2-20232A?style=for-the-badge&logo=ros
[ROS2-url]: https://docs.ros.org/en/iron/index.html
[Python-shield]: https://img.shields.io/badge/Python-20232A?style=for-the-badge&logo=python
[Python-url]: https://www.python.org/
[C-shield]: https://img.shields.io/badge/C-20232A?style=for-the-badge&logo=C
[C-url]: https://devdocs.io/c/
[Bash-shield]: https://img.shields.io/badge/Bash-20232A?style=for-the-badge&logo=gnubash
[Bash-url]: https://www.gnu.org/software/bash/
