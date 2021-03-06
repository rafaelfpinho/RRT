# -*- coding: utf-8 -*-
import math,random,sys,collections,re
from math import *
import pygame
#from shapely.geometry import Point, mapping
import time
import json

GeoPoint = collections.namedtuple('GeoPoint', 'latitude, longitude, altitude')
CartesianPoint = collections.namedtuple('CartesianPoint', 'x, y, z')

#Mostra graficamente a rota e obstaculos se setado em True, caso contrário setar como False
graph = False					

#Paarametros do algoritmo
XDIM = 200
YDIM = 200
SIZEX = 0.1
SIZEY = 0.1
k = 0.1
WINSIZE = [2*XDIM, 2*YDIM]
JUMP = 5.0
RADIUS = 10.0
NEIGHBORHOOD = 10.0
NUMNODES = 10000
white = 255, 240, 200
black = 20, 20, 40
red = 255, 0, 0
green = 15, 220, 182
home = GeoPoint(-12.825397,-50.349937,0)
execution_time = 180			#Duration of algorithym in seconds







#Node class for rrt
class Node():
	def __init__(self,point,parent,cost):
		self.point = point
		self.parent = parent
		self.cost = cost

	def get_point(self):
		return self.point

	def get_x(self):
		return self.point[0]

	def get_y(self):
		return self.point[1]

	def get_parent(self):
		return self.parent

	def set_parent(self,newparent):
		self.parent = newparent

	def get_cost(self):
		return self.cost

	def set_cost(self,newcost):
		self.cost = newcost

#Distance between two points
def dist(p1,p2):
	return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))


#Creates a point inside a line from two other points
def step_from_to(p1,p2):
	if dist(p1,p2) < JUMP:
		return p2
	else:
		theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
		return p1[0] + JUMP*cos(theta), p1[1] + JUMP*sin(theta)


#main function that does the code
def main(num_map):

	#gets obstacles from map file
	if num_map<10:
		map_file = "JSON/map_0"+str(num_map)+".json"
	else:
		map_file = "JSON/map_"+str(num_map)+".json"
	obstacles = map_json(map_file)

	#Initializes start and end point
	initPoint = (0,0)
	goalPoint = (0,-10)

	#Transformada cartesiana para que o mapa caiba no espaço de busca do algoritmo
	distx = initPoint[0]
	disty = initPoint[1]
	initPoint  = ((initPoint[0] - distx)/SIZEX,(initPoint[1] - disty)/SIZEY)
	goalPoint = ((goalPoint[0] - distx)/SIZEX,(goalPoint[1] - disty)/SIZEY)

	obs_graph = []
	obs = []

	#Trasnformada cartesiana dos obstáculos
	for j in range(len(obstacles)):
		for l in range(len(obstacles[j])):
			obstacles[j][l] = ((obstacles[j][l][0]-distx)/SIZEX,(obstacles[j][l][1]-disty)/SIZEY)
			obs_graph.append( (int(obstacles[j][l][0]+(XDIM)),int(obstacles[j][l][1]+(YDIM))) ) 
		obs.append(obs_graph)
		obs_graph = []



	print("inicio "+str(initPoint[0])+" , "+str(initPoint[1]))
	print("final "+str(goalPoint[0])+" , "+str(goalPoint[1]))

	if graph:
		pygame.init()
		screen = pygame.display.set_mode(WINSIZE)
		pygame.display.set_caption('RRT')
		screen.fill(black)
		pygame.draw.circle(screen,red,(int(initPoint[0]+(XDIM)),int(initPoint[1])+(YDIM)),2,2)
		pygame.draw.circle(screen,red,(int(goalPoint[0]+(XDIM)),int(goalPoint[1])+(YDIM)),int(RADIUS),int(RADIUS))
		for i in range(len(obstacles)):
			#print "obstacle = ",obstacles[i]
			pygame.draw.polygon(screen,green,obs[i])
		pygame.display.update()
	first_time = 1000
	init = time.time()
	nodes = []
	initNode = Node(initPoint,None,0)	#initializes initial node
	bestNode = Node((0,0),None,9999999)	#initializes best node
	node1 = Node((0,0),None,dist(initNode.get_point(),(0,0)))
	nodes.append(initNode)				#Puts initial node in the start of the tree
	var = True
	while var:
		#rand = random.random()*XDIM*pos_neg() , random.random()*YDIM*pos_neg()
		rand = (random.random()*XDIM*2)-XDIM , (random.random()*YDIM*2)-YDIM
		nn = nodes[0]
		for p in nodes:
			if dist(p.get_point(),rand) < dist(nn.get_point(),rand):
				nn = p 
				parentNode = p
		newnode = step_from_to(nn.get_point(),rand)
		node1 = Node(newnode,nn,nn.get_cost()+dist(newnode,nn.get_point()))
		for t in nodes:
			if dist(t.get_point(),node1.get_point())<NEIGHBORHOOD and node1.get_cost()>t.get_cost()+dist(node1.get_point(),t.get_point()):
				node1.set_parent(t)
				node1.set_cost(t.get_cost()+dist(node1.get_point(),t.get_point()))
		colision = 0
		for q in range(len(obstacles)):
			'''if(dentro_fora(obstacles[q],node1.get_point())):
				colision = colision+1'''
			if not_colides(node1.get_x(),node1.get_y(),obstacles[q]):
				colision = colision+1

		if colision >= len(obstacles):
			nodes.append(node1)
			if dist(node1.get_point(),goalPoint) < RADIUS:
				if node1.get_cost()<bestNode.get_cost():
					bestNode = node1
					print("custo: "+str(bestNode.get_cost()))
					print("tempo: "+str(time.time() - init))
					if(first_time > (time.time()-init)):
						first_time = time.time()-init
						best_time = first_time
					else:
						best_time = time.time()-init

		colision = 0

		if graph:
			for e in pygame.event.get():
				if e.type == pygame.QUIT: #or (e.type == KEYUP and e.key == K_ESCAPE):
					sys.exit("Leaving because you requested it.")

		fim = time.time()
		if dist(bestNode.get_point(),goalPoint) < RADIUS and (fim-init)>execution_time:
			best_cost = bestNode.get_cost()+dist(bestNode.get_point(),goalPoint)
			linhas = []
			print("custo ideal: "+str(dist(initPoint,goalPoint)))
			print("custo real: "+str(bestNode.get_cost()+dist(bestNode.get_point(),goalPoint)))
			print("tempo de execucao: "+str(fim-init))
			

			return best_cost, first_time, best_time

		#prints if path is not found in time
		elif((fim-init)>execution_time):
			print("Caminho não encontrado")
			return -1,-1,-1


#converts string to cartesian point
def string_to_cart(text):
	vetor = []
	vetor = text.split(',')
	point = []
	point.append(float(vetor[0]))
	point.append(float(vetor[1]))
	return point



#creates fix obstacles for testing
def obstacles_fix():
	points = [(200,10), (200,210), (240,210), (240,10)]
	return points

#verify if the route colides with the obstacle
def not_colides(x,y,poly):

	n = len(poly)
	inside = True

	p1x,p1y = poly[0]
	for i in range(n+1):
		p2x,p2y = poly[i % n]
		if y > min(p1y,p2y):
			if y <= max(p1y,p2y):
				if x <= max(p1x,p2x):
					if p1y != p2y:
						xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
					if p1x == p2x or x <= xints:
						inside = not inside
		p1x,p1y = p2x,p2y
	return inside



#converts geograpfic to cartesian
def to_cartesian(geo_point, home):
	x = calc_x(geo_point[1], home[1], home[0])
	y = calc_y(geo_point[0], home[0])

	#return CartesianPoint(x, y, geo_point.altitude)
	return (x,y)

#calculates y from geo reference
def calc_y(lat, lat_):
	return (lat - lat_) * (10000000.0 / 90)

#calculates x from geo reference
def calc_x(longi, longi_, lat_):
	pi = math.pi
	return (longi - longi_) * (6400000.0 * (math.cos(lat_ * pi / 180) * 2 * pi / 360)) # ToDo: verificar math

#converts cartesian to geographic
def to_geo_point(cartesian_point, home):
	longitude_x = calc_longitude_x(home.latitude, home.longitude, cartesian_point.x)
	latitude_y = calc_latitude_y(home.latitude, cartesian_point.y)

	return GeoPoint(latitude_y, longitude_x, cartesian_point.z)

def calc_latitude_y(lat_, y):
	return ((y * 90) / 10000000.0) + lat_


def calc_longitude_x(lat_, longi_, x):
	return ((x * 90) / (10008000 * math.cos(lat_ * math.pi / 180))) + longi_






def pos_neg():
	aleatorio = random.random()
	if(aleatorio>=0.5):
		return 1
	else:
		return -1


def map_json(map_file):
	mapa = open(map_file,'r')
	content = mapa.read()
	y = json.loads(content)
	coordinates = []
	x = json.dumps(y["areas_nao_navegaveis"])
	z = json.loads(x)

	for i in range(len(z)):
		coordinates.append(z[i]["cartesian_points"])

	for l in range(len(coordinates)):
		for m in range(len(coordinates[l])):
			aux = coordinates[l][m][1]
			coordinates[l][m][0] = coordinates[l][m][0]
			coordinates[l][m][1] = aux
			del(coordinates[l][m][2])

	mapa.close()
	return coordinates

def map_sgl(map_file):
	mapa = open(map_file,'r')
	content = mapa.readlines()
	num_poligonos = int(content[1])
	del(content[0])
	del(content[0])
	obstacles = []
	poligono = []
	x = []
	y = []

	for i in range(len(content)):
		if(i%3 == 0):
			text = content[i].split(", ")
			num_lados = int(text[2][4])
		if(i%3 == 1):
			text = content[i].split(",")
			for j in range(num_lados):
				x.append(float(text[j]))
		if(i%3 == 2):
			text = content[i].split(",")
			for j in range(num_lados):
				y.append(float(text[j]))
				poligono.append([x[j],y[j]])
			obstacles.append(poligono)
			x = []
			y = []
			poligono = []

	mapa.close()
	return obstacles

def dentro_fora(poligono, ponto):
	dentro = 1
	fora = 0
	for i in range(len(poligono)-1):
		h = vetor_normal(poligono[i],poligono[i+1])
		b = produto_escalar(h,poligono[i])
		if(b < produto_escalar(h,ponto)):
			return dentro
	return fora

def produto_escalar(v1, v2):
	return v1[0]*v2[0] + v1[1]*v2[1]

def vetor_normal(p1, p2):
	vetor = [p2[0]-p1[0],p2[1]-p1[1]]
	matriz_rotacao = [[0,-1],[1,0]]
	a = [-vetor[1],vetor[0]]
	versor_a = math.sqrt(a[0]**2 + a[1]**2)
	h = [a[0]/versor_a,a[1]/versor_a]
	return h

if __name__ == '__main__':
	arquivo_dados = open('arquivo_de_dados.txt','r')
	conteudo = arquivo_dados.readlines()
	arquivo_dados = open('arquivo_de_dados.txt','w')
	arquivo_dados.writelines(conteudo)
	for n in range(46,51):
		arquivo_dados.write('mapa '+str(n)+'\n')
		for i in range(3):
			arquivo_dados.write('teste '+str(i+1)+'\n')
			dados = main(n)
			print(dados)
			arquivo_dados.write("custo: ")
			arquivo_dados.write(str(dados[0])+"\n")
			arquivo_dados.write("tempo primeira rota:\n")
			arquivo_dados.write(str(dados[1])+"\n")
			arquivo_dados.write("tempo melhor rota:\n")
			arquivo_dados.write(str(dados[2])+"\n")


	arquivo_dados.close()
	