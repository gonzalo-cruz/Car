void espera(int time){
  for(int i = 0; i <= time; i++);
}

unsigned int calcular_distancia(unsigned short time){
  unsigned int distancia = 0;
  distancia = time*343;
//control the overflow
  return distancia;
}
