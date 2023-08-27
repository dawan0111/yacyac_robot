import {
  Controller,
  Get,
  Post,
  Body,
  Patch,
  Param,
  Delete,
} from '@nestjs/common';
import { HistorysService } from './historys.service';
import { CreateHistoryDto } from './dto/create-history.dto';
import { UpdateHistoryDto } from './dto/update-history.dto';
import { RobotsService } from 'src/robots/robots.service';

@Controller('historys')
export class HistorysController {
  constructor(
    private readonly historysService: HistorysService,
    private readonly robotsService: RobotsService,
  ) {}

  @Post()
  async create(@Body() createHistoryDto: CreateHistoryDto) {
    const [historyResult] = await Promise.all([
      this.historysService.create(createHistoryDto),
      createHistoryDto.accept
        ? this.robotsService.supplyUpdate({
            pill1: createHistoryDto.pill1,
            pill2: createHistoryDto.pill2,
            pill3: createHistoryDto.pill3,
            pill4: createHistoryDto.pill4,
            pill5: createHistoryDto.pill5,
            pill6: createHistoryDto.pill6,
            pill7: createHistoryDto.pill7,
            pill8: createHistoryDto.pill8,
          })
        : new Promise((resolve) => resolve(1)),
    ]);

    return historyResult;
  }

  @Get()
  async findAll() {
    return await this.historysService.findAll();
  }

  @Get('stat')
  async getStat() {
    return await this.historysService.getStat();
  }

  @Get(':id')
  async findOne(@Param('id') id: string) {
    return await this.historysService.findOne(+id);
  }

  @Patch(':id')
  async update(
    @Param('id') id: string,
    @Body() updateHistoryDto: UpdateHistoryDto,
  ) {
    return await this.historysService.update(+id, updateHistoryDto);
  }

  @Delete(':id')
  remove(@Param('id') id: string) {
    return this.historysService.remove(+id);
  }
}
