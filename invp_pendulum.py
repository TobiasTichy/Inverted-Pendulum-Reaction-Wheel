import pygame
import pygame.gfxdraw
import numpy as np
import random
import math
from scipy.linalg import solve_continuous_are

render_scale_factor = 4 #scaling factor for anti aliasing

class Pendulum:
    def __init__(self, screen, refresh):
        #pendulum specs
        self.alpha = np.deg2rad(10)
        self.omega = 0
        self.mass = 0.15 #kg
        self.L_real = 10 * 0.01 #centimeters
        self.moi = 1/3 * self.mass * self.L_real**2
        
        #graphics
        self.width = 10
        self.height = 150
        self.screen = screen
        self.refresh = refresh
        self.top_x = self.screen.get_width()//2 - self.height*np.sin(math.radians(self.alpha))
        self.top_y = self.screen.get_height()//2 - self.height*np.cos(math.radians(self.alpha))
        self.render_scale_factor = render_scale_factor
        
        #wheel
        self.wheel_radius = 75
        self.wheel = Wheel(screen, self.wheel_radius, self.top_x, self.top_y)
        self.max_torque = 68 * 0.001 #mNm
        
        #other
        self.t_elapsed = 0
        self.total_error = -self.alpha
        
        #state space, linearized around 0
        self.x = np.array([self.alpha, self.omega, self.wheel.alpha_wheel, self.wheel.omega_wheel])
        self.A = np.array([
            [0,                                         1,0,0],
            [self.mass * 9.81 * self.L_real/self.moi,   0,0,0],
            [0,                                         0,0,1],
            [0,                                         0,0,0]
        ], dtype=float)
        self.B = np.array([[0], [-1.0/self.moi], [0], [1.0/self.wheel.moi_wheel]], dtype=float)
        self.Q = np.diag([1,0.001,0.001,0.001])
        self.R = np.array([[10]])
        self.P = solve_continuous_are(self.A,self.B,self.Q,self.R)
        self.K = np.linalg.inv(self.R) @ self.B.T @ self.P
        print(np.linalg.eig(self.A-self.B@self.K)[0])
        
    def PID(self, alpha, omega, t, e_prev):
        e_total = e_prev - alpha
        kp = 0.01*abs(alpha)
        kd = 0.1
        ki = 0
        sat = 90
        if abs(alpha) < sat: 
            P = kp * -alpha
            #I = ki * e_total * t
            D = kd * -omega
        else:
            P = kp * -alpha/abs(alpha)* sat
            #I = ki * e_total * t
            D = kd * -omega
        I = 0
        return P + I + D, e_total
        
    def LQR(self, x):
        u_max = self.max_torque
        u = float(-self.K @ self.x)
        u = np.clip(u, -u_max, u_max)
        alpha_dot       = self.omega
        omega_dot       = self.mass * 9.81 * self.L_real * np.sin(self.alpha)/self.moi - u/self.moi
        alpha_wheel_dot = self.wheel.omega_wheel
        omega_wheel_dot = u/self.wheel.moi_wheel
        x_dot = np.array([alpha_dot, omega_dot, alpha_wheel_dot, omega_wheel_dot])
        return x_dot
        
    def update(self):
        x_dot = self.LQR(self.x)
        self.x = self.x + x_dot * self.refresh
        self.alpha, self.omega, self.wheel.alpha_wheel, self.wheel.omega_wheel = self.x
        
        self.wheel.x = self.screen.get_width()//2 - self.height*np.sin(self.alpha)
        self.wheel.y = self.screen.get_height()//2 - self.height*np.cos(self.alpha)

        self.t_elapsed += self.refresh
        
    def render(self):
        self.update()
        #draw rects
        pygame.draw.line(self.screen, (255, 255, 255), (0, self.screen.get_height() // 2), (self.screen.get_width(), self.screen.get_height() // 2)) 
        pygame.draw.line(self.screen, (255, 255, 255), (self.screen.get_width() // 2, 0), (self.screen.get_width() // 2, self.screen.get_height())) 
        rect_surface = pygame.Surface((self.width * self.render_scale_factor, self.height*2 * self.render_scale_factor), pygame.SRCALPHA)
        pygame.draw.rect(rect_surface, (255, 255, 255), (0,0, self.width*self.render_scale_factor, self.height*self.render_scale_factor))
        rotated_surface = pygame.transform.rotate(rect_surface, np.rad2deg(self.alpha))
        rotated_surface = pygame.transform.smoothscale(rotated_surface,(rotated_surface.get_width() // self.render_scale_factor,rotated_surface.get_height() // self.render_scale_factor))
        rotated_rect = rotated_surface.get_rect(center=(self.screen.get_width() // 2, self.screen.get_height() // 2))
        self.screen.blit(rotated_surface, rotated_rect)
        self.wheel.render()
        
        
class Wheel:
    def __init__(self, screen, radius, x, y):
        self.screen = screen
        self.radius = radius
        self.x = x
        self.y = y
        self.mass_ring = 0.3 #kg
        self.thickness = 10
        self.R_real = 5 * 0.01 #centimeters
        self.d_real = 0.5 * 0.01 #centimeters
        self.omega_wheel = 0
        self.alpha_wheel = np.deg2rad(0)
        self.moi_wheel = 0.5*(self.R_real**2 + (self.R_real - self.d_real)**2)*self.mass_ring
        #anti aliasing
        self.render_scale_factor = render_scale_factor

    def render(self):
        circle_surface = pygame.Surface((2*self.radius*self.render_scale_factor, 2*self.radius*self.render_scale_factor), pygame.SRCALPHA)
        pygame.draw.circle(circle_surface, (255,255,255), (self.radius*self.render_scale_factor, self.radius*self.render_scale_factor), self.radius*self.render_scale_factor, width = self.thickness*self.render_scale_factor)
        pygame.draw.rect(circle_surface, (255,255,255), (0*self.render_scale_factor, (self.radius - 2)*self.render_scale_factor, self.radius*2*self.render_scale_factor, 4*self.render_scale_factor))
        rotated_surface = pygame.transform.rotate(circle_surface, np.rad2deg(self.alpha_wheel))
        rotated_surface = pygame.transform.smoothscale(rotated_surface,(rotated_surface.get_width() // self.render_scale_factor,rotated_surface.get_height() // self.render_scale_factor))
        rotated_circle = rotated_surface.get_rect(center=(self.x,self.y))
        self.screen.blit(rotated_surface, rotated_circle)
        self.alpha_wheel += self.omega_wheel
        
        
        