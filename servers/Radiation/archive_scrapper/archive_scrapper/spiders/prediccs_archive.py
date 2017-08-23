import scrapy


class PrediccsArchiveScrapper(scrapy.Spider):

    flag = 0
    count = 0
    string = 'bryn/31daysMars.plot'
    name = "prediccs"
    links = []
    handle_httpstatus_list = [404]

    def start_requests(self):
        url = 'http://prediccs.sr.unh.edu/data/goesPlots/archive/'
        yield scrapy.Request(url=url, callback=self.parse)

    def parse(self, response):

        if response.status == 404:
            self.count = self.count + 1
            scrap_url = 'http://prediccs.sr.unh.edu/' + \
                        'data/goesPlots/archive/' + \
                        self.links[0][self.count] + self.string
            yield scrapy.Request(scrap_url, self.parse)

        if not self.flag:
            self.flag = 1
            linkobj = response.css("a")[5:-2]
            self.links.append(linkobj.css("a::attr(href)").extract())
            if self.count < len(self.links[0]):
                self.count = self.count + 1
                scrap_url = 'http://prediccs.sr.unh.edu/' + \
                            'data/goesPlots/archive/' + \
                            self.links[0][self.count] + self.string
                yield scrapy.Request(scrap_url, self.parse)

        datas = response.css("p::text").extract_first()
        datas = datas.split("\n")[22:]
        data = []
        for i in datas:
            i = i.split('\t')
            d = i[:6]
            d.append(i[-2])
            data.append(d)
        yield{
            'data': data,
        }
        if self.count < len(self.links[0]):
            self.count = self.count + 1
            scrap_url = 'http://prediccs.sr.unh.edu/' + \
                        'data/goesPlots/archive/' + \
                        self.links[0][self.count] + self.string
            yield scrapy.Request(scrap_url, self.parse)
