#include "planning_node.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
/**
 * @brief Exemplo simples de planejamento em C++ que consulta o mapa.
 *
 * Esta classe verifica o mapa global antes de aceitar um objetivo. Se livre,
 * implementa um algoritmo básico: vira em direção ao objetivo,
 * anda em linha reta se alinhado, e para se houver obstáculo à frente.
 *
 * Thresholds configuráveis:
 * - DIST_THRESHOLD: Distância para considerar objetivo atingido.
 * - ANGLE_THRESHOLD: Ângulo para considerar alinhado.
 * - OBSTACLE_THRESHOLD: Distância para detectar obstáculo.
 */
class PlanningExample : public PlanningNode
{
public:
  PlanningExample() : PlanningNode("planning_example_cpp") {
  }

private:
  rclcpp::TimerBase::SharedPtr marker_timer_;

  /**
   * @brief Exibe uma árvore de exemplo (RRT, etc) no RViz.
   *
   * Utiliza as configurações já estabelecidas na classe base (PlanningNode)
   * para marcações de nós (SPHERE_LIST) e arestas (LINE_LIST).
   * Neste exemplo, a função limpa os pontos anteriores e preenche 
   * novos pontos usando as coordenadas globais e absolutas baseadas 
   * na posição atual do robô, simulando um nó de origem e 3 nós filhos.
   */
  void publish_tree()
  {
    visualization_msgs::msg::MarkerArray marker_array;

    // Limpa os pontos anteriores
    nodes_marker_.points.clear();
    edges_marker_.points.clear();

    // Definindo os pontos (em coordenadas globais do mapa)
    geometry_msgs::msg::Point p0, p1, p2, p3;
    
    // Nó 0 na posição atual do robô
    p0.x = x_; 
    p0.y = y_; 
    p0.z = 0.0;
    
    // Nó 1 à frente (1 metro na direção theta)
    p1.x = x_ + 1.0 * cos(theta_); 
    p1.y = y_ + 1.0 * sin(theta_); 
    p1.z = 0.0;
    
    // Nó 2 a 45 graus para a esquerda
    p2.x = x_ + 1.0 * cos(theta_ + M_PI / 4.0);
    p2.y = y_ + 1.0 * sin(theta_ + M_PI / 4.0);
    p2.z = 0.0;

    // Nó 3 a 45 graus para a direita
    p3.x = x_ + 1.0 * cos(theta_ - M_PI / 4.0);
    p3.y = y_ + 1.0 * sin(theta_ - M_PI / 4.0);
    p3.z = 0.0;

    // Adicionando os nós na lista de esferas
    nodes_marker_.points.push_back(p0);
    nodes_marker_.points.push_back(p1);
    nodes_marker_.points.push_back(p2);
    nodes_marker_.points.push_back(p3);

    // Adicionando as arestas (cada par de pontos representa uma linha: p0->p1, p0->p2, p0->p3)
    edges_marker_.points.push_back(p0);
    edges_marker_.points.push_back(p1);
    
    edges_marker_.points.push_back(p0);
    edges_marker_.points.push_back(p2);

    edges_marker_.points.push_back(p0);
    edges_marker_.points.push_back(p3);

    marker_array.markers.push_back(nodes_marker_);
    marker_array.markers.push_back(edges_marker_);

    marker_pub_->publish(marker_array);
  }

  /**
   * @brief Callback chamado automaticamente ao receber um novo objetivo (goal) do RViz.
   *
   * Verifica se as coordenadas do objetivo estão dentro dos limites do mapa e
   * correspondem a uma célula livre de obstáculos.
   */
  void on_goal() override
  {
    // Se não há um objetivo válido armazenado, ignora
    if (!has_goal()) return;
    
    // Se o mapa global ainda não foi recebido, não é possível validar o destino
    if (!map_msg_) {
      RCLCPP_WARN(this->get_logger(), "Novo goal recebido (nenhum mapa recebido ainda), ignorando.");
      clear_goal();
      stop();
      return;
    }

    // Obtém as coordenadas (em metros) do objetivo
    double gx = std::get<0>(goal_);
    double gy = std::get<1>(goal_);
    
    // Obtém os parâmetros do mapa (resolução, posição de origem, dimensões da grade)
    double res = map_msg_->info.resolution;
    double origin_x = map_msg_->info.origin.position.x;
    double origin_y = map_msg_->info.origin.position.y;
    int width = map_msg_->info.width;
    int height = map_msg_->info.height;

    // Converte as coordenadas físicas do mundo (x, y) para índices espaciais (coluna, linha) na matriz do mapa
    int col = (int)((gx - origin_x) / res);
    int row = (int)((gy - origin_y) / res);

    // Confirma se o nó/célula alvo está dentro das margens conhecidas do mapa
    if (col >= 0 && col < width && row >= 0 && row < height) {
      // Como map_msg_->data é um vetor 1D, calcula o índice linear
      int idx = row * width + col;
      int val = map_msg_->data[idx];
      
      // Verifica o custo de ocupação da célula:
      // - 0 indica totalmente livre
      // - 100 indica totalmente ocupado
      // - -1 indica área desconhecida
      // Consideramos > 50 como obstáculo
      if (val > 50 || val == -1) {
        RCLCPP_WARN(this->get_logger(), "Goal ocupado ou desconhecido (custo=%d). Ignorando...", val);
        clear_goal(); // Reseta o objetivo se for inválido
        stop();       // Garante que o robô não tente ir para um ponto perigoso
      } else {
        RCLCPP_INFO(this->get_logger(), "Novo goal recebido e e livre (custo=%d)! Iniciando...", val);
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Goal fora dos limites do mapa. Ignorando...");
      clear_goal();
      stop();
    }
  }

  /**
   * @brief Loop de controle executado a 20 Hz.
   *
   * Implementa a lógica de planejamento local: verifica objetivo, calcula erros,
   * decide entre virar, andar ou parar.
   */
  void control_loop() override
  {
    if (!has_goal()) {
      stop();
      return;
    }

    double dist_to_goal = distance_to_goal();
    double angle_err = angle_to_goal();
    double front_dist = get_front_distance();

    // Thresholds
    const double DIST_THRESHOLD = 0.1;     // metros
    const double ANGLE_THRESHOLD = 0.3;    // radianos (~17 graus)
    const double OBSTACLE_THRESHOLD = 0.5; // metros

    if (dist_to_goal < DIST_THRESHOLD) {
      // Objetivo alcançado
      clear_goal();
      stop();
      RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
      return;
    }

    if (front_dist < OBSTACLE_THRESHOLD) {
      // Obstáculo à frente, parar
      stop();
      RCLCPP_WARN(this->get_logger(), "Obstáculo à frente, parando.");
      return;
    }

    if (std::abs(angle_err) > ANGLE_THRESHOLD) {
      // Virar em direção ao objetivo
      double w = (angle_err > 0) ? 0.5 : -0.5; // velocidade angular
      publish_velocity(0.0, w);
    } else {
      // Andar em linha reta
      publish_velocity(0.2, 0.0); // velocidade linear
    }

    publish_tree();
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningExample>());
  rclcpp::shutdown();
  return 0;
}