from constants import BACKGROUND_COLOR, BOARD_SIZE, BOARD_START_X, BOARD_START_Y, BORDER_WIDTH, BUTTON_COLOR, CELL_SIZE, COMPUTER_TEXT, EMPTY_COLOR, EMPTY_SYMBOL, END_TEXT_COLOR, HIT_COLOR, HIT_SYMBOL, HUMAN_TEXT, MISS_COLOR, MISS_SYMBOL, MOUSE_BUTTON_DOWN, PLAY_BUTTON_HEIGHT, PLAY_BUTTON_WIDTH, PLAY_BUTTON_X_OFFSET, PLAY_BUTTON_Y_OFFSET, SCREEN_HEIGHT, SCREEN_WIDTH, SHIP_BUTTON_HEIGHT, SHIP_BUTTON_HORIZONTAL_X, SHIP_BUTTON_VERTICAL_X, SHIP_BUTTON_WIDTH, SHIP_BUTTON_Y_INCREMENT, SHIP_COLOR, SHIP_INITIAL_Y, SHIP_SYMBOL, TEXT_COLOR, TEXT_OFFSET, TITLE_POSITION_RATIO, VIDEO_RESIZE
import pygame

from services.game_service import GameService


class GUI:
    def __init__(self):
        self.init_pygame()

        self.font = pygame.font.Font(None, 36)
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
        self.current_screen = WelcomeScreen(self.screen, self.font) 

        self.is_game_running = True
        self.game_service = GameService()

    def init_pygame(self):
        pygame.init()
        pygame.font.init()
        pygame.display.set_caption("Battleship")

    def run(self):
        while self.is_game_running:
            for pygame_event_type in pygame.event.get():
                if pygame_event_type.type == pygame.QUIT:
                    self.is_game_running = False
                elif pygame_event_type.type == VIDEO_RESIZE:
                    width = max(pygame_event_type.w, SCREEN_WIDTH)
                    height = max(pygame_event_type.h, SCREEN_HEIGHT)
                    self.screen = pygame.display.set_mode((width, height), pygame.RESIZABLE)

                result = self.current_screen.handle_event(pygame_event_type)
                if result == "welcome":
                    self.current_screen = WelcomeScreen(self.screen, self.font)
                elif result == "place_ships":
                    self.current_screen = PlaceShipsScreen(self.screen, self.font, self.game_service)
                elif result == "play":
                    self.current_screen = PlayScreen(self.screen, self.font,self.game_service)
                elif result == "end":
                    end_message = "You win!" if self.current_screen.winner == "player" else "You lose!"
                    self.current_screen = EndScreen(self.screen, self.font, end_message)

            self.current_screen.render()
            pygame.display.flip()

class ScreenBase:
    def __init__(self, screen, font):
        self.screen = screen
        self.font = font

    def handle_event(self, event):
        pass

    def render(self):
        pass

    def update(self):
        pass

class WelcomeScreen(ScreenBase):
    def __init__(self, screen, font):
        super().__init__(screen, font)
        self.screen_background_image = pygame.image.load('src/img/welcome.jpg')
        self.title_text = 'Battleships'
        self.button_text = 'Play'
        self.update_screen_elements()

    def update_screen_elements(self):
        self.background = pygame.transform.scale(self.screen_background_image, (self.screen.get_width(), self.screen.get_height()))

        self.title = self.font.render(self.title_text, True, TEXT_COLOR)
        self.title_rectangle = self.title.get_rect(center=(self.screen.get_width() // 2, self.screen.get_height() // TITLE_POSITION_RATIO))

        self.play_button = pygame.Rect(self.screen.get_width() // 2 - PLAY_BUTTON_X_OFFSET, self.screen.get_height() // 2 - PLAY_BUTTON_Y_OFFSET, PLAY_BUTTON_WIDTH, PLAY_BUTTON_HEIGHT)
        self.play_text = self.font.render(self.button_text, True, TEXT_COLOR)
        self.play_text_rectangle = self.play_text.get_rect(center=self.play_button.center)
        
    def handle_event(self, event):
        if event.type == MOUSE_BUTTON_DOWN and self.play_button.collidepoint(event.pos):
            return "place_ships"
        elif event.type == VIDEO_RESIZE:
            self.update_screen_elements()
        return "welcome"

    def render(self):
        self.screen.blit(self.background, (0, 0))

        self.screen.blit(self.title, self.title_rectangle.topleft)

        pygame.draw.rect(self.screen, BUTTON_COLOR, self.play_button)
        self.screen.blit(self.play_text, self.play_text_rectangle.topleft)


class PlaceShipsScreen(ScreenBase):
    def __init__(self, screen, font, game_service):
        super().__init__(screen, font)
        self.game_service = game_service
        self.original_background = pygame.image.load('src/img/welcome.jpg')
        self.selected_ship = None
        self.selected_orientation = None
        self.placing_ship_text = None
        self.error_message = None
        self.update_screen_elements()


    def update_screen_elements(self):
        self.background = pygame.transform.scale(self.original_background, (self.screen.get_width(), self.screen.get_height()))

        self.ship_buttons = []
        ship_y = SHIP_INITIAL_Y
        for ship_name, ship_size in self.game_service.ship_types_and_sizes.items():
            button_rect_horizontal = pygame.Rect(SHIP_BUTTON_HORIZONTAL_X, ship_y, SHIP_BUTTON_WIDTH, SHIP_BUTTON_HEIGHT)
            button_rect_vertical = pygame.Rect(SHIP_BUTTON_VERTICAL_X, ship_y, SHIP_BUTTON_WIDTH, SHIP_BUTTON_HEIGHT)
            self.ship_buttons.append((button_rect_horizontal, ship_name, ship_size, 'h'))
            self.ship_buttons.append((button_rect_vertical, ship_name, ship_size, 'v'))
            ship_y += SHIP_BUTTON_Y_INCREMENT

        self.board_start = (BOARD_START_X, BOARD_START_Y)
        self.cell_size = CELL_SIZE

        self.ships_placed = {ship_name: False for ship_name in self.game_service.ship_types_and_sizes}


    def handle_event(self, event):
        if self.ships_placed and all(self.ships_placed.values()):
            return "play"

        if event.type == MOUSE_BUTTON_DOWN:
            if self.selected_ship and not self.ships_placed[self.selected_ship]:
                for button_rectangle, ship_name, _, orientation in self.ship_buttons:
                    if ship_name == self.selected_ship and button_rectangle.collidepoint(event.pos):
                        self.selected_orientation = orientation
                        self.placing_ship_text = f"Placing {ship_name} ({orientation})"
                        break

                board_x, board_y = self.board_start
                for row in range(BOARD_SIZE):
                    for column in range(BOARD_SIZE):
                        cell_rectangle = pygame.Rect(board_x + column * self.cell_size, board_y + row * self.cell_size, self.cell_size, self.cell_size)
                        if cell_rectangle.collidepoint(event.pos):
                            try:
                                self.game_service.place_ship_for_human_player(self.selected_ship, (row, column), self.selected_orientation)
                                self.ships_placed[self.selected_ship] = True
                                self.selected_ship = None
                                self.selected_orientation = None
                                self.placing_ship_text = None
                                self.error_message = None
                            except Exception as exception_error_message:
                                self.error_message = str(exception_error_message) 
            else:
                for button_rectangle, ship_name, _, orientation in self.ship_buttons:
                    if button_rectangle.collidepoint(event.pos) and not self.ships_placed[ship_name]:
                        self.selected_ship = ship_name
                        self.selected_orientation = orientation
                        self.placing_ship_text = f"Placing {ship_name} ({orientation})"
                        break


    def render(self):
        self.screen.blit(self.background, (0, 0))

        if self.error_message:
            error_text_surface = self.font.render(self.error_message, True, (255, 0, 0))
            self.screen.blit(error_text_surface, (50, self.screen.get_height() - 40))

        
        for button_rectangle, ship_name, ship_size, orientation in self.ship_buttons:
            if not self.ships_placed[ship_name]:
                if self.selected_ship == ship_name and self.selected_orientation == orientation:
                    color = BUTTON_COLOR
                else:
                    color = (180, 180, 180)
                
                pygame.draw.rect(self.screen, color, button_rectangle)
                orientation_text = "H" if orientation == 'h' else "V"
                button_text_surface = self.font.render(f"{ship_size} - {orientation_text}", True, TEXT_COLOR)
                self.screen.blit(button_text_surface, (button_rectangle.x + (button_rectangle.width - button_text_surface.get_width()) / 2,
                                                       button_rectangle.y + (button_rectangle.height - button_text_surface.get_height()) / 2))
        
        board_representation = self.game_service.get_board_representation(is_human_player=True)
        board_x, board_y = self.board_start
        for row_index, row in enumerate(board_representation):
            for column_index, board_cell in enumerate(row):
                cell_rectangle = pygame.Rect(board_x + column_index * self.cell_size, board_y + row_index * self.cell_size, self.cell_size, self.cell_size)
                if board_cell == SHIP_SYMBOL: 
                    pygame.draw.rect(self.screen, SHIP_COLOR, cell_rectangle) 
                elif board_cell == HIT_SYMBOL: 
                    pygame.draw.rect(self.screen, HIT_COLOR, cell_rectangle)
                elif board_cell == MISS_SYMBOL: 
                    pygame.draw.rect(self.screen, MISS_COLOR, cell_rectangle)
                else:
                    pygame.draw.rect(self.screen, EMPTY_COLOR, cell_rectangle, 1)
                    

        if self.placing_ship_text:
            placing_text_surface = self.font.render(self.placing_ship_text, True, TEXT_COLOR)
            self.screen.blit(placing_text_surface, (50, 10))

class PlayScreen(ScreenBase):
    def __init__(self, screen, font, game_service):
        super().__init__(screen, font)
        self.game_service = game_service
        self.original_background = pygame.image.load('src/img/welcome.jpg') 
        self.is_player_turn = True 
        self.game_over = False
        self.update_screen_elements()
        self.game_service.init_game()
        self.winner = None
        self.cheat_mode = False
        self.cheat_button = pygame.Rect(self.screen.get_width() - 150, self.screen.get_height() - 40, 100, 30)

    def update_screen_elements(self):
        self.background = pygame.transform.scale(self.original_background, (self.screen.get_width(), self.screen.get_height()))

        self.player_board_start = (50, 50)
        self.computer_board_start = (self.screen.get_width() // 2 + 50, 50)
        self.cell_size = 30

    def handle_event(self, event):
        if self.game_over:
            return "end"

        if self.is_player_turn == False:
            self.game_service.computer_attack()
            if self.game_service.check_if_game_over():
                self.winner = "computer"
                self.game_over = True
            self.is_player_turn = True 
            return


        if event.type == MOUSE_BUTTON_DOWN:
            if self.cheat_button.collidepoint(event.pos):
                self.cheat_mode = not self.cheat_mode  
                return

            if self.is_player_turn:
                board_x, board_y = self.computer_board_start
                for row in range(BOARD_SIZE):
                    for col in range(BOARD_SIZE):
                        cell_rect = pygame.Rect(board_x + col * self.cell_size, board_y + row * self.cell_size, self.cell_size, self.cell_size)
                        if cell_rect.collidepoint(event.pos):
                            self.game_service.add_human_attack_against_computer((row, col))
                            if self.game_service.check_if_game_over():
                                self.winner = "player"
                                self.game_over = True
                            else:
                                self.is_player_turn = False 
                            return
                
    def render(self):
        self.screen.blit(self.background, (0, 0))

        self.draw_board(self.player_board_start, is_human_player=True)

        self.draw_board(self.computer_board_start, is_human_player=False, cheating=self.cheat_mode)

        pygame.draw.rect(self.screen, BUTTON_COLOR, self.cheat_button)
        cheat_text = self.font.render('Cheat', True, TEXT_COLOR)
        self.screen.blit(cheat_text, (self.cheat_button.x + 5, self.cheat_button.y + 5))


    def draw_board(self, board_start, is_human_player, cheating=False):
        board_representation = self.game_service.get_board_representation(is_human_player, cheating)
        board_x, board_y = board_start

        for row_index, row in enumerate(board_representation):
            for column_index, board_cell in enumerate(row):
                cell_rectangle = pygame.Rect(board_x + column_index * self.cell_size, board_y + row_index * self.cell_size, self.cell_size, self.cell_size)
                
                if board_cell == SHIP_SYMBOL:
                    pygame.draw.rect(self.screen, SHIP_COLOR, cell_rectangle)
                elif board_cell == HIT_SYMBOL:
                    pygame.draw.rect(self.screen, HIT_COLOR, cell_rectangle)
                elif board_cell == MISS_SYMBOL:
                    pygame.draw.rect(self.screen, MISS_COLOR, cell_rectangle)
                else:
                    pygame.draw.rect(self.screen, EMPTY_COLOR, cell_rectangle, BORDER_WIDTH)

        display_text = HUMAN_TEXT if is_human_player else COMPUTER_TEXT
        text_surface = self.font.render(display_text, True, TEXT_COLOR)

        text_x = board_x
        text_y = board_y + len(board_representation) * self.cell_size

        self.screen.blit(text_surface, (text_x, text_y))



class EndScreen(ScreenBase):
    def __init__(self, screen, font, message):
        super().__init__(screen, font)
        self.message = message
        self.background = pygame.Surface((screen.get_width(), screen.get_height()))
        self.background.fill(BACKGROUND_COLOR) 

    def render(self):
        self.screen.blit(self.background, (0, 0))

        message_surface = self.font.render(self.message, True, END_TEXT_COLOR)
        message_x = self.screen.get_width() // 2 - message_surface.get_width() // 2
        message_y = self.screen.get_height() // 2 - message_surface.get_height() // 2
        self.screen.blit(message_surface, (message_x, message_y))
