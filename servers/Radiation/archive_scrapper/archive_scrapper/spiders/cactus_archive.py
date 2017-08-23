import scrapy


class CactusArchiveScrapper(scrapy.Spider):

    name = "cactus"
    count = 0
    flag = 0
    links = []

    def start_requests(self):

        url = 'https://secchi.nrl.navy.mil/cactus/'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):

        if not self.flag:
            self.flag = 1
            linkobj = response.css('li')[2:24]
            self.links.append(linkobj.css("a::attr(href)").extract())
            if self.count < len(self.links[0]):
                self.count = self.count + 1
                yield scrapy.Request(self.links[0][self.count], self.parse)
        table = response.css("pre::text")
        data = []
        for i in range(len(table) - 2):
            datas = table[i + 2].extract()
            data.append(datas.split('|')[1:9])
            if 'Flow' in datas:
                break
        yield{
            'data': data,
        }
        if self.count < len(self.links[0]):
            self.count = self.count + 1
            yield scrapy.Request(self.links[0][self.count], self.parse)
