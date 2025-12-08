"use client"

import { useEffect, useRef } from "react"
import * as THREE from "three"
import gsap from "gsap"

export function ThreeDRover() {
  const containerRef = useRef<HTMLDivElement>(null)
  const sceneRef = useRef<THREE.Scene | null>(null)
  const robotGroupRef = useRef<THREE.Group | null>(null)

  useEffect(() => {
    if (!containerRef.current) return

    // Scene setup
    const scene = new THREE.Scene()
    sceneRef.current = scene
    scene.background = new THREE.Color(0x0a0e0f)

    const camera = new THREE.PerspectiveCamera(
      75,
      containerRef.current.clientWidth / containerRef.current.clientHeight,
      0.1,
      1000,
    )
    camera.position.set(2.5, 1.8, 2.5)
    camera.lookAt(0, 0, 0)

    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
    renderer.setSize(containerRef.current.clientWidth, containerRef.current.clientHeight)
    renderer.shadowMap.enabled = true
    containerRef.current.appendChild(renderer.domElement)

    // Lighting
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5)
    scene.add(ambientLight)

    const directionalLight = new THREE.DirectionalLight(0x00ff88, 0.8)
    directionalLight.position.set(5, 5, 5)
    directionalLight.castShadow = true
    scene.add(directionalLight)

    const pointLight = new THREE.PointLight(0x00d9ff, 0.5)
    pointLight.position.set(-5, 3, 0)
    scene.add(pointLight)

    // Create robot
    const robotGroup = new THREE.Group()
    robotGroupRef.current = robotGroup
    scene.add(robotGroup)

    // Body
    const bodyGeometry = new THREE.BoxGeometry(1, 1.2, 1.5)
    const bodyMaterial = new THREE.MeshPhongMaterial({
      color: 0x00ff88,
      emissive: 0x00ff88,
      emissiveIntensity: 0.3,
      shininess: 100,
    })
    const body = new THREE.Mesh(bodyGeometry, bodyMaterial)
    body.castShadow = true
    robotGroup.add(body)

    // Head/Turret
    const headGeometry = new THREE.BoxGeometry(0.6, 0.5, 0.8)
    const headMaterial = new THREE.MeshPhongMaterial({
      color: 0x00d9ff,
      emissive: 0x00d9ff,
      emissiveIntensity: 0.2,
    })
    const head = new THREE.Mesh(headGeometry, headMaterial)
    head.position.y = 0.9
    head.position.z = 0.4
    head.castShadow = true
    robotGroup.add(head)

    // Sensor/Camera
    const sensorGeometry = new THREE.SphereGeometry(0.15, 16, 16)
    const sensorMaterial = new THREE.MeshPhongMaterial({
      color: 0x00ff00,
      emissive: 0x00ff00,
      emissiveIntensity: 0.5,
    })
    const sensor = new THREE.Mesh(sensorGeometry, sensorMaterial)
    sensor.position.set(0, 0.9, 0.8)
    sensor.castShadow = true
    robotGroup.add(sensor)

    // Wheels
    const wheelGeometry = new THREE.CylinderGeometry(0.3, 0.3, 0.25, 16)
    const wheelMaterial = new THREE.MeshPhongMaterial({
      color: 0x001a00,
      emissive: 0x003300,
      emissiveIntensity: 0.1,
    })

    const wheels: THREE.Mesh[] = []
    const positions = [
      { x: -0.5, y: -0.6, z: 0.6 },
      { x: 0.5, y: -0.6, z: 0.6 },
      { x: -0.5, y: -0.6, z: -0.6 },
      { x: 0.5, y: -0.6, z: -0.6 },
    ]

    positions.forEach((pos) => {
      const wheel = new THREE.Mesh(wheelGeometry, wheelMaterial)
      wheel.rotation.z = Math.PI / 2
      wheel.position.set(pos.x, pos.y, pos.z)
      wheel.castShadow = true
      wheels.push(wheel)
      robotGroup.add(wheel)
    })

    // Ground
    const groundGeometry = new THREE.PlaneGeometry(10, 10)
    const groundMaterial = new THREE.MeshPhongMaterial({
      color: 0x0f1419,
      emissive: 0x001a00,
    })
    const ground = new THREE.Mesh(groundGeometry, groundMaterial)
    ground.rotation.x = -Math.PI / 2
    ground.position.y = -0.65
    ground.receiveShadow = true
    scene.add(ground)

    // Scanning rings (visual effect)
    const ringGroup = new THREE.Group()
    robotGroup.add(ringGroup)

    for (let i = 0; i < 3; i++) {
      const ringGeometry = new THREE.TorusGeometry(0.8 + i * 0.3, 0.05, 8, 16)
      const ringMaterial = new THREE.MeshBasicMaterial({
        color: new THREE.Color().setHSL(0.33, 1, 0.5), // Green hue
        transparent: true,
        opacity: 0.4 - i * 0.1,
      })
      const ring = new THREE.Mesh(ringGeometry, ringMaterial)
      ring.position.y = -0.2
      ringGroup.add(ring)
    }

    // Animation timeline
    const timeline = gsap.timeline()

    // Initial delay of 2 seconds, then start animations
    setTimeout(() => {
      // Robot idle floating animation
      gsap.to(robotGroup.position, {
        y: 0.1,
        duration: 2,
        repeat: -1,
        yoyo: true,
        ease: "sine.inOut",
      })

      // Continuous rotation
      gsap.to(robotGroup.rotation, {
        y: Math.PI * 2,
        duration: 8,
        repeat: -1,
        ease: "none",
      })

      // Wheel spinning
      wheels.forEach((wheel) => {
        gsap.to(wheel.rotation, {
          x: Math.PI * 2,
          duration: 0.8,
          repeat: -1,
          ease: "none",
        })
      })

      // Sensor pulsing
      gsap.to(sensorMaterial, {
        emissiveIntensity: 1,
        duration: 1.5,
        repeat: -1,
        yoyo: true,
        ease: "sine.inOut",
      })

      // Head rotation (scanning)
      gsap.to(head.rotation, {
        y: 0.3,
        duration: 2,
        repeat: -1,
        yoyo: true,
        ease: "sine.inOut",
      })

      // Ring rotation
      gsap.to(ringGroup.rotation, {
        z: Math.PI * 2,
        duration: 4,
        repeat: -1,
        ease: "none",
      })

      // Body material glow pulse
      gsap.to(bodyMaterial, {
        emissiveIntensity: [0.3, 0.6, 0.3],
        duration: 3,
        repeat: -1,
        ease: "sine.inOut",
      })
    }, 2000)

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate)
      renderer.render(scene, camera)
    }
    animate()

    // Handle resize
    const handleResize = () => {
      if (containerRef.current) {
        const width = containerRef.current.clientWidth
        const height = containerRef.current.clientHeight
        camera.aspect = width / height
        camera.updateProjectionMatrix()
        renderer.setSize(width, height)
      }
    }

    window.addEventListener("resize", handleResize)

    return () => {
      window.removeEventListener("resize", handleResize)
      containerRef.current?.removeChild(renderer.domElement)
    }
  }, [])

  return <div ref={containerRef} className="w-full h-80 rounded-lg" style={{ backgroundColor: "#0f1419" }} />
}
